<template>
    <div>
        <section class="section">
            <div class="container">
                <div class="buttons">
                    <button class="button is-medium is-outlined is-fullwidth" :class="autonomy.enabled ? 'is-success' : 'is-danger'" @click="toggleAutonomy()">
                        Autonomy {{ autonomy.enabled ? 'Enabled' : 'Disabled' }}
                    </button>
                </div>
            </div>
        </section>
        <section class="section">
            <div class="container">
                <div class="field">
                    <label class="label">Path Affinity</label>
                    <div class="control">
                        <input class="slider is-fullwidth is-large has-output"
                            name="path-affinity"
                            id="path-affinity-slider"
                            type="range"
                            min="1.0"
                            max="3.0"
                            step="0.1"
                            v-model="autonomy.pathAffinity"
                            @change="setPathAffinity()"
                        />
                        <output for="path-affinity-slider">{{ autonomy.pathAffinity }}</output>
                    </div>
                </div>
                <div class="field">
                    <label class="label">Max Overshoot</label>
                    <div class="control">
                        <input class="slider is-fullwidth is-large has-output"
                            name="max-overshoot"
                            id="max-overshoot-slider"
                            type="range"
                            min="0.0"
                            max="1.0"
                            step="0.1"
                            v-model="autonomy.mover.maxOvershoot"
                            @change="setMaxOvershoot()"
                        />
                        <output for="max-overshoot-slider">{{ autonomy.mover.maxOvershoot }}</output>
                    </div>
                </div>
                <div class="field">
                    <ValidationObserver v-slot="{ invalid }">
                        <label class="label">Learning Rate</label>
                        <ValidationProvider name="learning-rate" rules="required|double|min_value:0" v-slot="{ errors }">
                            <div class="control">
                                <input class="input is-fullwidth has-text-right"
                                    name="learning-rate"
                                    type="number"
                                    min="0"
                                    step="0.01"
                                    :class="{ 'is-danger' : invalid }"
                                    v-model="autonomy.mover.learningRate"
                                    @change="setLearningRate()"
                                />
                            </div>
                            <p class="help">{{ errors[0] }}</p>
                        </ValidationProvider>
                    </ValidationObserver>
                </div>
                <div class="field">
                    <label class="label">Momentum</label>
                    <div class="control">
                        <input class="slider is-fullwidth is-large has-output"
                            name="momentum"
                            id="momentum-slider"
                            type="range"
                            min="0.0"
                            max="1.0"
                            step="0.1"
                            v-model="autonomy.mover.momentum"
                            @change="setMomentum()"
                        />
                        <output for="momentum-slider">{{ autonomy.mover.momentum }}</output>
                    </div>
                </div>
                <div class="field">
                    <ValidationObserver v-slot="{ invalid }">
                        <label class="label">L2 Regularization</label>
                        <ValidationProvider name="alpha" rules="required|double|min_value:0" v-slot="{ errors }">
                            <div class="control">
                                <input class="input is-fullwidth has-text-right"
                                    name="alpha"
                                    type="number"
                                    min="0"
                                    step="0.0001"
                                    :class="{ 'is-danger' : invalid }"
                                    v-model="autonomy.mover.alpha"
                                    @change="setAlpha()"
                                />
                            </div>
                            <p class="help">{{ errors[0] }}</p>
                        </ValidationProvider>
                    </ValidationObserver>
                </div>
            </div>
        </section>
        <page-loader :loading="loading"></page-loader>
    </div>
</template>

<script lang="ts">
import Vue from 'vue';
import bus from '../providers/bus';

export default Vue.extend({
    data() {
        return {
            autonomy: {
                enabled: false,
                pathAffinity: 2.0,
                mover: {
                    learningRate: 0.1,
                    momentum: 0.9,
                    alpha: 1e-4,
                    maxOvershoot: 0.3,
                },
            },
            loading: false,
        };
    },
    methods: {
        toggleAutonomy() : void {
            if (this.autonomy.enabled) {
                this.$http.delete('/robot/autonomy').then(() => {
                    this.autonomy.enabled = false;

                    bus.$emit('autonomy-disabled');
                }).catch((error) => {
                    bus.$emit('communication-error', {
                        error,
                    });
                });
            } else {
                this.$http.put('/robot/autonomy/enabled').then(() => {
                    this.autonomy.enabled = true;

                    bus.$emit('autonomy-enabled');
                }).catch((error) => {
                    bus.$emit('communication-error', {
                        error,
                    });
                });
            }
        },
        setPathAffinity() : void {
            this.$http.put('/robot/autonomy/path-affinity', {
                pathAffinity: this.autonomy.pathAffinity,
            }).catch((error) => {
                bus.$emit('communication-error', {
                    error,
                });
            });
        },
        setMaxOvershoot() : void {
            this.$http.put('/robot/autonomy/mover/max-overshoot', {
                maxOvershoot: this.autonomy.mover.maxOvershoot,
            }).catch((error) => {
                bus.$emit('communication-error', {
                    error,
                });
            });
        },
        setLearningRate() : void {
            this.$http.put('/robot/autonomy/mover/learning-rate', {
                learningRate: this.autonomy.mover.learningRate,
            }).catch((error) => {
                bus.$emit('communication-error', {
                    error,
                });
            });
        },
        setMomentum() : void {
            this.$http.put('/robot/autonomy/mover/momentum', {
                momentum: this.autonomy.mover.momentum,
            }).catch((error) => {
                bus.$emit('communication-error', {
                    error,
                });
            });
        },
        setAlpha() : void {
            this.$http.put('/robot/autonomy/mover/alpha', {
                alpha: this.autonomy.mover.alpha,
            }).catch((error) => {
                bus.$emit('communication-error', {
                    error,
                });
            });
        },
    },
    mounted() {
        this.loading = true;

        this.$http.get('/robot').then((response) => {
            this.autonomy = response.data.robot.autonomy;

            this.loading = false;
        }).catch((error) => {
            bus.$emit('communication-error', {
                error,
            });
        });

        bus.$on('stopped', () => {
            this.autonomy.enabled = false;
        });
    },
});
</script>
