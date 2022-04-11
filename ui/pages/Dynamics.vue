<template>
    <div>
        <section class="section">
            <div class="container">
                <h2 class="title">Rotator</h2>
                <div class="field">
                    <label class="label">Proportional Gain</label>
                    <div class="control">
                        <input class="slider is-fullwidth is-large has-output"
                            name="rotator-p-gain"
                            id="rotator-p-gain-slider"
                            type="range"
                            min="0.0"
                            max="3.0"
                            step="0.1"
                            v-model="rotator.p"
                            @change="setRotatorPGain()"
                        />
                        <output for="rotator-p-gain-slider">{{ rotator.p }}</output>
                    </div>
                </div>
                <div class="field">
                    <label class="label">Integral Gain</label>
                    <div class="control">
                        <input class="slider is-fullwidth is-large has-output"
                            name="rotator-i-gain"
                            id="rotator-i-gain-slider"
                            type="range"
                            min="0.0"
                            max="3.0"
                            step="0.1"
                            v-model="rotator.i"
                            @change="setRotatorIGain()"
                        />
                        <output for="rotator-i-gain-slider">{{ rotator.i }}</output>
                    </div>
                </div>
                <div class="field">
                    <label class="label">Derivative Gain</label>
                    <div class="control">
                        <input class="slider is-fullwidth is-large has-output"
                            name="rotator-d-gain"
                            id="rotator-d-gain-slider"
                            type="range"
                            min="0.0"
                            max="3.0"
                            step="0.1"
                            v-model="rotator.d"
                            @change="setRotatorDGain()"
                        />
                        <output for="rotator-d-gain-slider">{{ rotator.d }}</output>
                    </div>
                </div>
            </div>
        </section>
        <section class="section">
            <div class="container">
                <h2 class="title">Stabilizer</h2>
                <div class="field">
                    <label class="label">Proportional Gain</label>
                    <div class="control">
                        <input class="slider is-fullwidth is-large has-output"
                            name="stabilizer-p-gain"
                            id="stabilizer-p-gain-slider"
                            type="range"
                            min="0.0"
                            max="3.0"
                            step="0.1"
                            v-model="stabilizer.p"
                            @change="setStabilizerPGain()"
                        />
                        <output for="stabilizer-p-gain-slider">{{ stabilizer.p }}</output>
                    </div>
                </div>
                <div class="field">
                    <label class="label">Integral Gain</label>
                    <div class="control">
                        <input class="slider is-fullwidth is-large has-output"
                            name="stabilizer-i-gain"
                            id="stabilizer-i-gain-slider"
                            type="range"
                            min="0.0"
                            max="3.0"
                            step="0.1"
                            v-model="stabilizer.i"
                            @change="setStabilizerIGain()"
                        />
                        <output for="stabilizer-i-gain-slider">{{ stabilizer.i }}</output>
                    </div>
                </div>
                <div class="field">
                    <label class="label">Derivative Gain</label>
                    <div class="control">
                        <input class="slider is-fullwidth is-large has-output"
                            name="stabilizer-d-gain"
                            id="stabilizer-d-gain-slider"
                            type="range"
                            min="0.0"
                            max="3.0"
                            step="0.1"
                            v-model="stabilizer.d"
                            @change="setStabilizerDGain()"
                        />
                        <output for="stabilizer-d-gain-slider">{{ stabilizer.d }}</output>
                    </div>
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
            rotator: {
                p: 0.0,
                i: 0.0,
                d: 0.0,
            },
            stabilizer: {
                p: 0.0,
                i: 0.0,
                d: 0.0,
            },
            loading: false,
        };
    },
    methods: {
        setRotatorPGain() : void {
            this.$http.put('/robot/rotator/p', {
                gain: this.rotator.p,
            }).catch((error) => {
                bus.$emit('communication-error', {
                    error,
                });
            });
        },
        setRotatorIGain() : void {
            this.$http.put('/robot/rotator/i', {
                gain: this.rotator.i,
            }).catch((error) => {
                bus.$emit('communication-error', {
                    error,
                });
            });
        },
        setRotatorDGain() : void {
            this.$http.put('/robot/rotator/d', {
                gain: this.rotator.d,
            }).catch((error) => {
                bus.$emit('communication-error', {
                    error,
                });
            });
        },
        setStabilizerPGain() : void {
            this.$http.put('/robot/stabilizer/p', {
                gain: this.stabilizer.p,
            }).catch((error) => {
                bus.$emit('communication-error', {
                    error,
                });
            });
        },
        setStabilizerIGain() : void {
            this.$http.put('/robot/stabilizer/i', {
                gain: this.stabilizer.i,
            }).catch((error) => {
                bus.$emit('communication-error', {
                    error,
                });
            });
        },
        setStabilizerDGain() : void {
            this.$http.put('/robot/stabilizer/d', {
                gain: this.stabilizer.d,
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
            const robot = response.data.robot;

            this.rotator.p = robot.rotator.p;
            this.rotator.i = robot.rotator.i;
            this.rotator.d = robot.rotator.d;

            this.stabilizer.p = robot.stabilizer.p;
            this.stabilizer.i = robot.stabilizer.i;
            this.stabilizer.d = robot.stabilizer.d;

            this.loading = false;
        }).catch((error) => {
            bus.$emit('communication-error', {
                error,
            });
        });
    },
});
</script>
