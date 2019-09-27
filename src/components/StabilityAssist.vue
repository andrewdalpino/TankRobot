<template>
    <div class="container">
        <h3 class="label">Stability Assist</h3>
        <div class="field">
            <div class="control">
                <input id="stabilize" type="checkbox" name="stabilize" class="switch is-primary" v-model="stabilize" @click="toggle()">
                <label for="stabilize">{{ stabilize ? 'Enabled' : 'Disabled' }}</label>
            </div>
        </div>
    </div>
</template>

<script>
    import bus from '../bus';

    export default {
        data() {
            return {
                stabilize: this.tank.stabilize,
            };
        },
        props: {
            tank: {
                type: Object,
                required: true,
            },
        },
        methods: {
            toggle() {
                if (this.stabilize) {
                    this.$http.delete('/movement/stabilizer').then((response) => {
                        bus.$emit('stabilizer-disabled');
                    }).catch((error) => {
                        bus.$emit('communication-error', {
                            error,
                        });

                        this.stabilize = true;
                    });
                } else {
                    this.$http.put('/movement/stabilizer').then((response) => {
                        bus.$emit('stabilizer-enabled');
                    }).catch((error) => {
                        bus.$emit('communication-error', {
                            error,
                        });

                        this.stabilize = false;
                    });
                }
            },
        },
    }
</script>
