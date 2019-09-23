<template>
    <div>
        <main-nav></main-nav>
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
                    stopped: true,
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

            bus.$on('throttle-set', (payload) => {
                this.tank.throttle = payload.throttle;
            });

            bus.$on('going', (payload) => {
                this.tank.stopped = false;
            });

            bus.$on('stopping', (payload) => {
                this.tank.stopped = true;
            });
        },
    }
</script>
