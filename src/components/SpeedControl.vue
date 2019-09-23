<template>
    <div class="container">
        <h3 class="label">Movement Speed</h3>
        <div class="field">
            <div class="control">
                <label for="speed">{{ speed }}</label>
                <input id="speed" class="slider is-fullwidth is-info" type="range" min="250" max="1000" step="1" v-model="speed" @change="setSpeed()">
            </div>
        </div>
    </div>
</template>

<script>
    import bus from '../bus';

    export default {
        data() {
            return {
                speed: this.tank.speed,
            };
        },
        props: {
            tank: {
                type: Object,
                required: true,
            },
        },
        methods: {
            setSpeed() {
                this.$http.put('/movement/speed', {
                    speed: this.speed,
                }).then((response) => {
                    bus.$emit('movement-speed-set', {
                        speed: this.speed,
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