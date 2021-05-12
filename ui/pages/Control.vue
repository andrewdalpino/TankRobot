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
            robot: {
                motors: undefined,
                sensors: undefined,
            },
            loading: false,
        };
    },
    mounted() {
        this.loading = true;

        this.$http.get('/robot').then((response) => {
            this.robot.motors = response.data.robot.motors;
            this.robot.sensors = response.data.robot.sensors;

            this.$sse('/events/robot/sensors', { format: 'json' }).then((sse) => {
                sse.subscribe('battery-voltage-updated', (event) => {
                    this.robot.sensors.battery.voltage = event.voltage;
                    this.robot.sensors.battery.percentage = event.percentage;
                });

                sse.subscribe('temperature-updated', (event) => {
                    this.robot.sensors.temperature = event.temperature;
                });
            });

            this.loading = false;
        }).catch((error) => {
            bus.$emit('communication-error', {
                error,
            });
        });

        bus.$on('moving-forward', () => {
            this.robot.motors.direction = FORWARD;
            this.robot.motors.stopped = false;
        });

        bus.$on('moving-left', () => {
            this.robot.motors.direction = LEFT;
            this.robot.motors.stopped = false;
        });

        bus.$on('moving-right', () => {
            this.robot.motors.direction = RIGHT;
            this.robot.motors.stopped = false;
        });

        bus.$on('moving-reverse', () => {
            this.robot.motors.direction = REVERSE;
            this.robot.motors.stopped = false;
        });

        bus.$on('stopped', () => {
            this.robot.motors.direction = null;
            this.robot.motors.stopped = true;
        });

        bus.$on('throttle-updated', (event) => {
            this.robot.motors.throttle = event.throttle;
        });
        
        bus.$on('autonomy-enabled', () => {
            this.robot.motors.direction = null;
            this.robot.motors.stopped = true;
        });
    },
});
</script>