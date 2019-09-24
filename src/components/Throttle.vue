<template>
    <div class="container">
        <h3 class="label">Throttle</h3>
        <div class="field">
            <div class="control has-text-centered">
                <input id="throttle" class="slider is-fullwidth is-large is-link" type="range" min="0" max="100" step="1" orient="vertical" v-model="percentage" @change="setThrottle()">
                <label for="throttle">{{ percentage }}%</label>
            </div>
        </div>
    </div>
</template>

<script>
    import bus from '../bus';

    const MIN_THROTTLE = 250;
    const MAX_THROTTLE = 1000;

    export default {
        data() {
            return {
                percentage: Math.round(((this.tank.throttle - MIN_THROTTLE) * 100) / (MAX_THROTTLE - MIN_THROTTLE)),
            }
        },
        props: {
            tank: {
                type: Object,
                required: true,
            },
        },
        methods: {
            setThrottle() {
                const throttle = (this.percentage * (MAX_THROTTLE - MIN_THROTTLE) / 100) + MIN_THROTTLE;

                this.$http.put('/throttle', {
                    throttle,
                }).then((response) => {
                    bus.$emit('throttle-set', {
                        throttle,
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