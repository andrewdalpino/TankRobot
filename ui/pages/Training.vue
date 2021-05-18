<template>
    <div>
        <section class="section">
            <div class="container">
                <div class="columns">
                    <div class="column is-half">
                        <h2 class="title is-size-4">Feature Importances</h2>
                        <figure class="image is-square">
                            <div id="feature-importances-chart" class="has-ratio"></div>
                        </figure>
                    </div>
                    <div class="column is-half">
                        <h2 class="title is-size-4">Training Loss</h2>
                        <figure class="image is-square">
                            <div id="training-loss-chart" class="has-ratio"></div>
                        </figure>
                    </div>
                </div>
            </div>
        </section>
        <page-loader :loading="loading"></page-loader>
    </div>
</template>

<script lang="ts">
import Vue from 'vue';
import Plotly from '../providers/plotly';
import bus from '../providers/bus';

const DATASET_SIZE = 80;

export default Vue.extend({
    data() {
        return {
            loading: false,
        };
    },
    mounted() {
        this.loading = true;

        this.$http.get('/robot').then((response) => {
            this.$sse('/events/robot/training', { format: 'json' }).then((sse) => {
                Plotly.newPlot('feature-importances-chart', [
                    {
                        values: Object.values(response.data.robot.autonomy.mover.importances),
                        labels: ['Throttle', 'Battery', 'Pitch', 'Distance', 'Average Distance'],
                        type: 'pie'
                    }
                ], {
                    margin: {
                        l: 32,
                        r: 32,
                        t: 32,
                        b: 32,
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
                    modeBarButtons: [
                        ['zoom2d', 'pan2d', 'resetScale2d', 'toImage'],
                    ],
                });

                Plotly.newPlot('training-loss-chart', [
                    {
                        name: 'Mover',
                        x: Array(DATASET_SIZE).fill(0),
                        y: Array(DATASET_SIZE).fill(0.0),
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
                        l: 32,
                        r: 32,
                        t: 32,
                        b: 32,
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
                    modeBarButtons: [
                        ['zoom2d', 'pan2d', 'resetScale2d', 'toImage'],
                    ],
                });

                sse.subscribe('mover-epoch-complete', (event) => {
                    Plotly.react('feature-importances-chart', {
                        values: Object.values(event.importances),
                        labels: ['Throttle', 'Battery', 'Pitch', 'Distance', 'Average Distance'],
                        type: 'pie'
                    });

                    Plotly.extendTraces('training-loss-chart', {x: [[event.epoch]], y: [[event.loss]]}, [0], DATASET_SIZE);
                });

                this.loading = false;
            });
        }).catch((error) => {
            bus.$emit('communication-error', {
                error,
            });
        });
    },
});
</script>
