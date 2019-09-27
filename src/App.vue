<template>
    <div>
        <main-nav :tank="tank"></main-nav>
        <main>
            <router-view :tank="tank"></router-view>
        </main>
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
                    throttle: 750,
                    direction: '',
                    stabilize: true,
                    stopped: true,
                    battery_voltage: 8.4,
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

            this.$options.sockets.onmessage = (message) => {
                let data = JSON.parse(message.data);

                switch (data.name) {
                    case 'battery-update':
                        this.tank.battery_voltage = data.battery_voltage;

                        break;

                    case 'temperature-update':
                        this.tank.temperature = data.temperature;

                        break;
                }
            }

            bus.$on('moving', (payload) => {
                this.tank.direction = payload.direction;
                this.tank.stopped = false;
            });

            bus.$on('throttle-set', (payload) => {
                this.tank.throttle = payload.throttle;
            });

            bus.$on('stabilizer-enabled', (payload) => {
                this.tank.stabilize = true;
            });

            bus.$on('stabilizer-disabled', (payload) => {
                this.tank.stabilize = false;
            });

            bus.$on('stopping', (payload) => {
                this.tank.stopped = true;
                this.tank.direction = '';
            });
        },
    }
</script>
