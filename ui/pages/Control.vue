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
            motors: {
                direction: null,
                throttle: undefined,
                stopped: true,
            },
            sensors: {
                lidar: {
                    visibility: undefined,
                },
                battery: {
                    voltage: undefined,
                    capacity: undefined,
                },
                temperature: undefined,
            },
            loading: false,
        };
    },
    methods: {
        handleMovingForward() : void {
            this.motors.direction = FORWARD;
            this.motors.stopped = false;
        },
        handleMovingLeft() : void {
            this.motors.direction = LEFT;
            this.motors.stopped = false;
        },
        handleMovingRight() : void {
            this.motors.direction = RIGHT;
            this.motors.stopped = false;
        },
        handleMovingReverse() : void {
            this.motors.direction = REVERSE;
            this.motors.stopped = false;
        },
        handleStopped() : void {
            this.motors.direction = null;
            this.motors.stopped = true;
        },
        handleThrottleUpdated(event) : void {
            this.motors.throttle = event.throttle;
        },
        handleBatteryVoltageUpdated(event) : void {
            this.sensors.battery.voltage = event.voltage;
            this.sensors.battery.capacity = event.capacity;
        },
        handleVisibilityUpdated(event) : void {
            this.sensors.lidar.visibility = event.visibility;
        },
        handleTemperatureUpdated(event) : void {
            this.sensors.temperature = event.temperature;
        },
        handleAutonomyEnabled() : void {
            this.motors.direction = null;
            this.motors.stopped = true;
        },
    },
    mounted() {
        this.loading = true;

        this.$http.get('/robot').then((response) => {
            const robot = response.data.robot;

            this.motors.direction = robot.motors.direction;
            this.motors.throttle = robot.motors.throttle;
            this.motors.stopped = robot.motors.stopped;

            this.sensors.lidar.visibility = robot.sensors.lidar.visibility;

            this.sensors.battery.voltage = robot.sensors.battery.voltage;
            this.sensors.battery.capacity = robot.sensors.battery.capacity;

            this.sensors.temperature = robot.sensors.temperature;

            this.$sse('/events/robot/sensors', { format: 'json' }).then((sse) => {
                sse.subscribe('visibility-updated', this.handleVisibilityUpdated);
                sse.subscribe('battery-voltage-updated', this.handleBatteryVoltageUpdated);
                sse.subscribe('temperature-updated', this.handleTemperatureUpdated);

                this.loading = false;
            });
        }).catch((error) => {
            bus.$emit('communication-error', {
                error,
            });
        });

        bus.$on('moving-forward', this.handleMovingForward);
        bus.$on('moving-left', this.handleMovingLeft);
        bus.$on('moving-right', this.handleMovingRight);
        bus.$on('moving-reverse', this.handleMovingReverse);
        bus.$on('stopped', this.handleStopped);
        bus.$on('throttle-updated', this.handleThrottleUpdated);
        bus.$on('autonomy-enabled', this.handleAutonomyEnabled);
    },
});
</script>