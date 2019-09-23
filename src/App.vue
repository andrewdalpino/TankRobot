<template>
    <div>
        <main-nav></main-nav>
        <main>
            <router-view :tank="tank"></router-view>
        </main>
        <communication-error></communication-error>
        <audio id="plucky" src="/plucky.ogg"></audio>
    </div>
</template>

<script>
    import bus from './bus';

    export default {
        data() {
            return {
                tank: {
                    speed: 250,
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

            bus.$on('movement-speed-set', (payload) => {
                this.tank.speed = payload.speed;
            });
        },
    }
</script>
