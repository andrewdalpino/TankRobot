<template>
    <div class="container">
        <h3 class="label">Direction</h3>
        <div class="field has-text-centered">
            <a class="button is-large" :class="tank.direction === 'forward' && !tank.stopped ? 'is-success' : 'is-info'" @click="forward()">
                <span class="icon">
                    <i class="fas fa-chevron-circle-up"></i>
                </span>
            </a>
        </div>
        <div class="field is-grouped is-grouped-centered">
            <p class="control">
                <a class="button is-large" :class="tank.direction === 'left' && !tank.stopped ? 'is-success' : 'is-info'" @click="left()">
                    <span class="icon">
                        <i class="fas fa-chevron-circle-left"></i>
                    </span>
                </a>
            </p>
            <p class="control">
                <a class="button is-large" :class="tank.stopped ? 'is-danger' : 'is-link'" @click="stop()">
                    <span class="icon">
                        <i class="fas fa-stop-circle"></i>
                    </span>
                </a>
            </p>
            <p class="control">
                <a class="button is-large" :class="tank.direction === 'right' && !tank.stopped ? 'is-success' : 'is-info'"  @click="right()">
                    <span class="icon">
                        <i class="fas fa-chevron-circle-right"></i>
                    </span>
                </a>
            </p>
        </div>
        <div class="field has-text-centered">
            <a class="button is-large" :class="tank.direction === 'reverse' && !tank.stopped ? 'is-success' : 'is-info'" @click="reverse()">
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
                this.$http.put('/movement/forward').then((response) => {
                    bus.$emit('moving', {
                        direction: 'forward',
                    });
                }).catch((error) => {
                    bus.$emit('communication-error', {
                        error,
                    });
                });
            },
            left() {
                this.$http.put('/movement/left').then((response) => {
                    bus.$emit('moving', {
                        direction: 'left',
                    });
                }).catch((error) => {
                    bus.$emit('communication-error', {
                        error,
                    });
                });
            },
            right() {
                this.$http.put('/movement/right').then((response) => {
                    bus.$emit('moving', {
                        direction: 'right',
                    });
                }).catch((error) => {
                    bus.$emit('communication-error', {
                        error,
                    });
                });;
            },
            reverse() {
                this.$http.put('/movement/reverse').then((response) => {
                    bus.$emit('moving', {
                        direction: 'reverse',
                    });
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
    }
</script>