<template>
    <div>
        <section class="section">
            <div class="container">
                <div class="buttons">
                    <button class="button is-medium is-outlined is-fullwidth" :class="robot.autonomy.enabled ? 'is-success' : 'is-danger'" @click="toggleAutonomy()">
                        <span class="icon"><i class="fas fa-brain"></i></span><span>Autonomy</span>
                    </button>
                </div>
            </div>
        </section>
        <section class="section">
            <h2 class="title is-size-4"><span class="icon mr-3"><i class="fas fa-chart-line"></i></span>Training Losses</h2>
            <figure class="image is-16by9">
                <div id="training-loss-chart" class="has-ratio"></div>
            </figure>
        </section>
        <page-loader :loading="loading"></page-loader>
    </div>
</template>

<script lang="ts">
import Vue from 'vue';
import Plotly from '../providers/plotly';
import bus from '../providers/bus';

const DATASET_SIZE = 100;

export default Vue.extend({
    data() {
        return {
            robot: {
                autonomy: {
                    enabled: false,
                }
            },
            loading: false,
        };
    },
    methods: {
        toggleAutonomy() : void {
            if (this.robot.autonomy.enabled) {
                this.$http.delete('/robot/features/autonomy').then(() => {
                    this.robot.autonomy.enabled = false;

                    bus.$emit('autonomy-disabled');
                }).catch((error) => {
                    bus.$emit('communication-error', {
                        error,
                    });
                });
            } else {
                this.$http.put('/robot/features/autonomy').then(() => {
                    this.robot.autonomy.enabled = true;

                    bus.$emit('autonomy-enabled');
                }).catch((error) => {
                    bus.$emit('communication-error', {
                        error,
                    });
                });
            }
        },
    },
    mounted() {
        this.loading = true;

        this.$http.get('/robot').then((response) => {
            this.robot.autonomy = response.data.robot.autonomy;

            this.$sse('/events/robot/training', { format: 'json' }).then((sse) => {
                sse.subscribe('mover-epoch-complete', (event) => {
                    Plotly.extendTraces('training-loss-chart', {y: [[event.loss]]}, [0], DATASET_SIZE);
                });
            });

            this.loading = false;
        }).catch((error) => {
            bus.$emit('communication-error', {
                error,
            });
        });

        Plotly.newPlot('training-loss-chart', [
            {
                name: 'Mover',
                x: [...Array(DATASET_SIZE).keys()].reverse(),
                y: Array(DATASET_SIZE).fill(0),
                type: 'scatter',
                line: {
                    width: 2,
                    color: 'rgb(184, 107, 255)',
                },
                fill: 'tozeroy',
                fillcolor: 'rgba(184, 107, 255, 0.1)',
            },
        ], {
            legend: {
                orientation: 'h',
                y: 1.2,
            },
            xaxis: {
                title: {
                    text: 'Epoch',
                    font: {
                        size: 12,
                    },
                },
                type: 'linear',
                autorange: 'reversed',
                gridcolor: 'rgb(128, 128, 128)',
            },
            yaxis: {
                title: {
                    text: 'L2 Loss',
                    font: {
                        size: 12,
                    },
                },
                type: 'linear',
                rangemode: 'tozero',
                gridcolor: 'rgb(128, 128, 128)',
                fixedrange: true,
            },
            margin: {
                l: 80,
                r: 40,
                t: 40,
                b: 40,
            },
            paper_bgcolor: 'rgba(0, 0, 0, 0)',
            plot_bgcolor: 'rgba(0, 0, 0, 0)',
            modebar: {
                color: 'rgb(128, 128, 128)',
                activecolor: 'rgb(192, 192, 192)',
                bgcolor: 'rgba(0, 0, 0, 0)',
            },
        }, {
            responsive: true,
            displaylogo: false,
            displayModeBar: true,
        });
    },
});
</script>
