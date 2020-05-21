<template>
    <div class="container">
        <h3 class="label">Direction</h3>
        <div class="field has-text-centered">
            <a class="button is-large" :class="direction === 'forward' && !stopped ? 'is-success' : 'is-info'" @click="forward()">
                <span class="icon">
                    <i class="fas fa-chevron-circle-up"></i>
                </span>
            </a>
        </div>
        <div class="field is-grouped is-grouped-centered">
            <p class="control">
                <a class="button is-large" :class="direction === 'left' && !stopped ? 'is-success' : 'is-info'" @click="left()">
                    <span class="icon">
                        <i class="fas fa-chevron-circle-left"></i>
                    </span>
                </a>
            </p>
            <p class="control">
                <a class="button is-large" :class="stopped ? 'is-danger' : 'is-link'" @click="stop()">
                    <span class="icon">
                        <i class="fas fa-stop-circle"></i>
                    </span>
                </a>
            </p>
            <p class="control">
                <a class="button is-large" :class="direction === 'right' && !stopped ? 'is-success' : 'is-info'"  @click="right()">
                    <span class="icon">
                        <i class="fas fa-chevron-circle-right"></i>
                    </span>
                </a>
            </p>
        </div>
        <div class="field has-text-centered">
            <a class="button is-large" :class="direction === 'reverse' && !stopped ? 'is-success' : 'is-info'" @click="reverse()">
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
        data() {
            return {
                direction: undefined,
                stopped: true,
            }
        },
        props: {
            tank: {
                type: Object,
                required: true,
            },
        },
        methods: {
            forward() {
                if (this.direction !== 'forward') {
                    this.$http.put('/movement/forward').catch((error) => {
                        bus.$emit('communication-error', {
                            error,
                        });
                    });
                }

                this.direction = 'forward';
                this.stopped = false;
            },
            left() {
                if (this.direction !== 'left') {
                    this.$http.put('/movement/left').catch((error) => {
                        bus.$emit('communication-error', {
                            error,
                        });
                    });

                    this.direction = 'left';
                    this.stopped = false;
                }
            },
            right() {
                if (this.direction !== 'right') {
                    this.$http.put('/movement/right').catch((error) => {
                        bus.$emit('communication-error', {
                            error,
                        });
                    });

                    this.direction = 'right';
                    this.stopped = false;
                }
            },
            reverse() {
                if (this.direction !== 'reverse') {
                    this.$http.put('/movement/reverse').catch((error) => {
                        bus.$emit('communication-error', {
                            error,
                        });
                    });

                    this.direction = 'reverse';
                    this.stopped = false;
                }
            },
            stop() {
                if (!this.stopped) {
                    this.$http.put('/movement/stop').catch((error) => {
                        bus.$emit('communication-error', {
                            error,
                        });
                    });

                    this.direction = undefined;
                    this.stopped = true;
                }
            },
        },
    }
</script>