<template>
    <div>
        <main-nav></main-nav>
        <main>
            <router-view :robot="robot"></router-view>
        </main>
        <rollover-detected></rollover-detected>
        <communication-error></communication-error>
        <page-loader :loading="loading"></page-loader>
        <audio id="plucky" src="./sounds/plucky.ogg" crossOrigin="anonymous"></audio>
    </div>
</template>

<script lang="ts">
import Vue from 'vue';
import { FORWARD, REVERSE, LEFT, RIGHT } from './direction_constants';
import bus from './bus';

export default Vue.extend({
    data() {
        return {
            robot: {
                motors: {
                    direction: null,
                    throttle: 0,
                    stopped: true,
                },
                sensors: {
                    voltage: null,
                    temperature: null,
                },
                features: {
                    autonomous: false,
                },
            },
            loading: false,
        };
    },
    mounted() {
        this.loading = true;

        this.$http.get('/robot').then((response) => {
            this.robot = response.data.robot;

            this.$sse('/robot/malfunctions/events', { format: 'json' }).then((sse) => {
                sse.subscribe('collision-detected', () => {
                    bus.$emit('collision-detected');
                });

                sse.subscribe('rollover-detected', () => {
                    bus.$emit('rollover-detected');
                });

                sse.subscribe('battery-undervoltage', (event) => {
                    bus.$emit('battery-undervoltage', {
                        voltage: event.voltage,
                    });
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

        bus.$on('battery-voltage-updated', (event) => {
            this.robot.sensors.voltage = event.voltage;
        });

        bus.$on('temperature-updated', (event) => {
            this.robot.sensors.temperature = event.temperature;
        });
    },
});
</script>