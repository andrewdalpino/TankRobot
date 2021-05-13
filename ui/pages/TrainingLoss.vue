<template>
    <div>
        <section class="section">
            <div class="container">
                <figure class="image is-16by9">
                    <div id="training-loss-chart" class="has-ratio"></div>
                </figure>
            </div>
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
            loading: false,
        };
    },
    mounted() {
        this.loading = true;

        this.$sse('/events/robot/training', { format: 'json' }).then((sse) => {
            sse.subscribe('mover-epoch-complete', (event) => {
                Plotly.extendTraces('training-loss-chart', {y: [[event.loss]]}, [0], DATASET_SIZE);
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
                t: 60,
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
            modeBarButtons: [
                ['zoom2d', 'pan2d', 'resetScale2d', 'toImage'],
            ],
        });
    },
});
</script>
