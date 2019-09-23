<template>
    <div class="container">
        <h3 class="label">Movement Direction</h3>
        <div class="field has-text-centered">
            <a class="button is-large is-primary" @click="forward()">
                <span class="icon">
                    <i class="fas fa-chevron-circle-up"></i>
                </span>
            </a>
        </div>
        <div class="field is-grouped is-grouped-centered">
            <p class="control">
                <a class="button is-large is-primary" @click="left()">
                    <span class="icon">
                        <i class="fas fa-chevron-circle-left"></i>
                    </span>
                </a>
            </p>
            <p class="control">
                <a class="button is-large is-primary" @click="stop()">
                    <span class="icon">
                        <i class="fas fa-stop-circle"></i>
                    </span>
                </a>
            </p>
            <p class="control">
                <a class="button is-large is-primary">
                    <span class="icon">
                        <i class="fas fa-chevron-circle-right" @click="right()"></i>
                    </span>
                </a>
            </p>
        </div>
        <div class="field has-text-centered">
            <a class="button is-large is-primary" @click="reverse()">
                <span class="icon">
                    <i class="fas fa-chevron-circle-down"></i>
                </span>
            </a>
        </div>
    </div>
</template>

<script>
    import bus from '../bus';

    export default {
        props: {
            tank: {
                type: Object,
                required: true,
            },
        },
        methods: {
            forward() {
                this.$http.put('/direction/forward').then((response) => {
                    this.go();
                }).catch((error) => {
                    bus.$emit('communication-error', {
                        error,
                    });
                });
            },
            left() {
                this.$http.put('/direction/left').then((response) => {
                    this.go();
                }).catch((error) => {
                    bus.$emit('communication-error', {
                        error,
                    });
                });
            },
            right() {
                this.$http.put('/direction/right').then((response) => {
                    this.go();
                }).catch((error) => {
                    bus.$emit('communication-error', {
                        error,
                    });
                });;
            },
            reverse() {
                this.$http.put('/direction/reverse').then((response) => {
                    this.go();
                }).catch((error) => {
                    bus.$emit('communication-error', {
                        error,
                    });
                });
            },
            go() {
                this.$http.put('/movement/go').then((response) => {
                    bus.$emit('going');
                }).catch((error) => {
                    bus.$emit('communication-error', {
                        error,
                    });
                });
            },
            stop() {
                this.$http.put('/movement/stop').then((response) => {
                    bus.$emit('stopping');
                }).catch((error) => {
                    bus.$emit('communication-error', {
                        error,
                    });
                });
            },
        },
        created() {
            bus.$on('throttle-set', (payload) => {
                if (!this.tank.stopped) {
                    this.go();
                }
            });
        },
    }
</script>