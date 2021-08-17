<template>
    <div>
        <section class="section">
            <div class="container">
                <gauge-level v-if="sensors" :sensors="sensors"></gauge-level>
            </div>
        </section>
        <section class="section">
            <div class="container">
                <div class="columns is-mobile">
                    <div class="column is-one-quarter">
                        <motor-throttle v-if="motors" :motors="motors"></motor-throttle>
                    </div>
                    <div class="column is-three-quarters">
                        <direction-pad v-if="motors" :motors="motors"></direction-pad>
                    </div>
                </div>
            </div>
        </section>
        <section class="section">
            <div class="container">
                <feature-panel></feature-panel>
            </div>
        </section>
        <page-loader :loading="loading"></page-loader>
    </div>
</template>

<script lang="ts">
import Vue from 'vue';
import { FORWARD, REVERSE, LEFT, RIGHT } from '../constants';
import bus from '../providers/bus';

export default Vue.extend({
    data() {
        return {
            motors: undefined,
            sensors: undefined,
            loading: false,
        };
    },
    mounted() {
        this.loading = true;

        this.$http.get('/robot').then((response) => {
            this.motors = response.data.robot.motors;
            this.sensors = response.data.robot.sensors;

            this.$sse('/events/robot/sensors', { format: 'json' }).then((sse) => {
                sse.subscribe('battery-voltage-updated', (event) => {
                    this.sensors.battery.voltage = event.voltage;
                    this.sensors.battery.percentage = event.percentage;
                });

                sse.subscribe('temperature-updated', (event) => {
                    this.sensors.temperature = event.temperature;
                });

                this.loading = false;
            });
        }).catch((error) => {
            bus.$emit('communication-error', {
                error,
            });
        });

        bus.$on('moving-forward', () => {
            this.motors.direction = FORWARD;
            this.motors.stopped = false;
        });

        bus.$on('moving-left', () => {
            this.motors.direction = LEFT;
            this.motors.stopped = false;
        });

        bus.$on('moving-right', () => {
            this.motors.direction = RIGHT;
            this.motors.stopped = false;
        });

        bus.$on('moving-reverse', () => {
            this.motors.direction = REVERSE;
            this.motors.stopped = false;
        });

        bus.$on('stopped', () => {
            this.motors.direction = null;
            this.motors.stopped = true;
        });

        bus.$on('throttle-updated', (event) => {
            this.motors.throttle = event.throttle;
        });
        
        bus.$on('autonomy-enabled', () => {
            this.motors.direction = null;
            this.motors.stopped = true;
        });
    },
});
</script>