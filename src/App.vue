<template>
    <div>
        <main-nav :tank="tank"></main-nav>
        <main>
            <router-view :tank="tank"></router-view>
        </main>
        <rollover-detected></rollover-detected>
        <communication-error></communication-error>
        <audio id="plucky" src="plucky.ogg"></audio>
    </div>
</template>

<script>
    import bus from './bus';

    export default {
        data() {
            return {
                tank: {
                    throttle: 0.0,
                    battery_voltage: 0.0,
                    temperature: 0.0,
                    position: {
                        lat: undefined,
                        lon: undefined,
                    },
                    num_satellites: 0,
                },
            };
        },
        created() {
            this.$http.get('/status').then((response) => {
                this.tank = response.data;
            }).catch((error) => {
                bus.$emit('communication-error', {
                    error,
                });
            });

            this.$sse('/events', { format: 'json' }).then((sse) => {
                sse.subscribe('status-update', (message, event) => {
                    this.tank.battery_voltage = message.battery_voltage;
                    this.tank.temperature = message.temperature;
                    this.tank.num_satellites = message.num_satellites;
                });

                sse.subscribe('position-update', (message, event) => {
                    this.tank.position.lat = message.lat;
                    this.tank.position.lon = message.lon;
                });

                sse.subscribe('rollover-detected', (message, event) => {
                    bus.$emit('rollover-detected');
                });
            });

            bus.$on('throttle-set', (payload) => {
                this.tank.throttle = payload.throttle;
            });
        },
    }
</script>
