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
                    throttle: 625,
                    direction: 'forward',
                    stopped: true,
                    battery_level: 1.0,
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

            this.$options.sockets.onmessage = (event) => {
                let data = JSON.parse(event.data);

                switch (data.name) {
                    case 'battery-update':
                        this.tank.battery_level = data.battery_level;

                        break;
                }
            }

            bus.$on('throttle-set', (payload) => {
                this.tank.throttle = payload.throttle;
            });

            bus.$on('moving', (payload) => {
                this.tank.direction = payload.direction;
                this.tank.stopped = false;
            });

            bus.$on('stopping', (payload) => {
                this.tank.stopped = true;
            });
        },
    }
</script>
