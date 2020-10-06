<template>
    <div class="container ">
        <h3 class="label">Throttle</h3>
        <div class="field">
            <div class="control has-text-centered">
                <input id="throttle" class="slider is-fullwidth is-large is-link" type="range" min="0" max="100" step="1" orient="vertical" v-model="throttle" @change="setThrottle()" />
            </div>
        </div>
    </div>
</template>

<script>
    import bus from '../bus';

    export default {
        data() {
            return {
                throttle: this.tank.throttle,
            };
        },
        props: {
            tank: {
                type: Object,
                required: true,
            },
        },
        methods: {
            setThrottle() {
                this.$http.put('/throttle', {
                    throttle: this.throttle,
                }).then((response) => {
                    bus.$emit('throttle-set', {
                        throttle: this.throttle,
                    });
                }).catch((error) => {
                    bus.$emit('communication-error', {
                        error,
                    });
                });
            },
        },
    }
</script>

<style lang="scss" scoped>
    #throttle {
        height: 180px;
    }
</style>