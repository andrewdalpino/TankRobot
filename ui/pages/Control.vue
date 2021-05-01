<template>
    <div>
        <section class="section">
            <div class="container">
                <gauge-level v-if="robot.sensors" :sensors="robot.sensors"></gauge-level>
            </div>
        </section>
        <section class="section">
            <div class="container">
                <div class="columns is-mobile">
                    <div class="column is-one-quarter">
                        <motor-throttle v-if="robot.motors" :motors="robot.motors"></motor-throttle>
                    </div>
                    <div class="column is-three-quarters">
                        <direction-pad v-if="robot.motors" :motors="robot.motors"></direction-pad>
                    </div>
                </div>
            </div>
        </section>
        <section class="section">
            <div class="container">
                <feature-panel v-if="robot.features" :features="robot.features"></feature-panel>
            </div>
        </section>
    </div>
</template>

<script lang="ts">
import Vue from 'vue';
import bus from '../bus';

export default Vue.extend({
    props: {
        robot: {
            type: Object,
            required: true,
        },
    },
    mounted() {
        this.$sse('/robot/sensors/events', { format: 'json' }).then((sse) => {
            sse.subscribe('battery-voltage-updated', (event) => {
                bus.$emit('battery-voltage-updated', {
                    voltage: event.voltage,
                });
            });

            sse.subscribe('temperature-updated', (event) => {
                bus.$emit('temperature-updated', {
                    temperature: event.temperature,
                });
            });
        });
    },
});
</script>