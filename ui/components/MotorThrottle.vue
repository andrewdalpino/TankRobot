<template>
    <div class="field">
        <div class="control has-text-centered">
            <input class="slider is-fullwidth is-large"
                name="throttle"
                type="range"
                min="0"
                max="100"
                step="1"
                orient="vertical"
                v-model="throttle"
                @change="setThrottle()"
            />
        </div>
    </div>
</template>

<script lang="ts">
import Vue from 'vue';
import bus from '../bus';

export default Vue.extend({
    data() {
        return {
            throttle: this.motors.throttle,
        };
    },
    props: {
        motors: {
            type: Object,
            required: true,
        },
    },
    methods: {
        setThrottle() {
            this.$http.put('/robot/motors/throttle', {
                throttle: this.throttle,
            }).then(() => {
                bus.$emit('throttle-updated', {
                    throttle: this.throttle,
                });
            }).catch((error) => {
                bus.$emit('communication-error', {
                    error,
                });
            });
        },
    },
});
</script>

<style lang="scss" scoped>
    input[name=throttle] {
        height: 180px;
        writing-mode: bt-lr;
        -webkit-appearance: slider-vertical;
    }
</style>
