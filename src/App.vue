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
                    direction: undefined,
                    stopped: undefined,
                    battery_voltage: 0.0,
                    temperature: 0.0,
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

            this.$sse('/events', { format: 'json' }).then(sse => {
                sse.subscribe('battery-update', (message, event) => {
                    this.tank.battery_voltage = message.battery_voltage;
                });

                sse.subscribe('temperature-update', (message, event) => {
                    this.tank.temperature = message.temperature;
                });

                sse.subscribe('rollover-detected', (message, event) => {
                    bus.$emit('rollover-detected');
                });
            });

            bus.$on('moving', (payload) => {
                this.tank.direction = payload.direction;
                this.tank.stopped = false;
            });

            bus.$on('throttle-set', (payload) => {
                this.tank.throttle = payload.throttle;
            });
            
            bus.$on('stopping', (payload) => {
                this.tank.stopped = true;
                this.tank.direction = undefined;
            });
        },
    }
</script>
