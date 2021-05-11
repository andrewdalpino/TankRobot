<template>
    <div>
        <div class="field is-grouped is-grouped-centered">
            <p class="control">
                <button class="button is-large" @click="rotateLeft()">
                    <span class="icon">
                        <i class="fas fa-chevron-circle-up" style="transform: rotate(315deg);"></i>
                    </span>
                </button>
            </p>
            <p class="control">
                <button class="button is-large" :class="isForward ? 'is-success' : 'is-info'" @mousedown="forward()" @mouseup="stop()">
                    <span class="icon">
                        <i class="fas fa-chevron-circle-up"></i>
                    </span>
                </button>
            </p>
            <p class="control">
                <button class="button is-large" @click="rotateRight()">
                    <span class="icon">
                        <i class="fas fa-chevron-circle-up" style="transform: rotate(45deg);"></i>
                    </span>
                </button>
            </p>
        </div>
        <div class="field is-grouped is-grouped-centered">
            <p class="control">
                <button class="button is-large" :class="isLeft ? 'is-success' : 'is-info'" @mousedown="left()" @mouseup="stop()">
                    <span class="icon">
                        <i class="fas fa-chevron-circle-left"></i>
                    </span>
                </button>
            </p>
            <p class="control">
                <button class="button is-large" :class="!isMoving ? 'is-danger' : 'is-link'" @click="stop()">
                    <span class="icon">
                        <i class="fas fa-stop-circle"></i>
                    </span>
                </button>
            </p>
            <p class="control">
                <button class="button is-large" :class="isRight ? 'is-success' : 'is-info'" @mousedown="right()" @mouseup="stop()">
                    <span class="icon">
                        <i class="fas fa-chevron-circle-right"></i>
                    </span>
                </button>
            </p>
        </div>
        <div class="field is-grouped is-grouped-centered">
            <p class="control">
                <button class="button is-large" @click="rotateRight()">
                    <span class="icon">
                        <i class="fas fa-chevron-circle-up" style="transform: rotate(225deg);"></i>
                    </span>
                </button>
            </p>
            <p class="control">
                <button class="button is-large" :class="isReverse ? 'is-success' : 'is-info'" @mousedown="reverse()" @mouseup="stop()">
                    <span class="icon">
                        <i class="fas fa-chevron-circle-down"></i>
                    </span>
                </button>
            </p>
            <p class="control">
                <button class="button is-large" @click="rotateLeft()">
                    <span class="icon">
                        <i class="fas fa-chevron-circle-up" style="transform: rotate(135deg);"></i>
                    </span>
                </button>
            </p>
        </div>
    </div>
</template>

<script lang="ts">
import Vue from 'vue';
import { FORWARD, REVERSE, LEFT, RIGHT } from '../constants';
import bus from '../bus';

export default Vue.extend({
    props: {
        motors: {
            type: Object,
            required: true,
        },
    },
    computed: {
        isForward() : boolean {
            return this.isMoving && this.motors.direction === FORWARD;
        },
        isLeft() : boolean {
            return this.isMoving && this.motors.direction === LEFT;
        },
        isRight() : boolean {
            return this.isMoving && this.motors.direction === RIGHT;
        },
        isReverse() : boolean {
            return this.isMoving && this.motors.direction === REVERSE;
        },
        isMoving() : boolean {
            return this.motors.stopped === false;
        },
    },
    methods: {
        forward() : void {
            if (!this.isForward) {
                this.$http.put('/robot/motors/direction', {
                    direction: FORWARD,
                }).then((response) => {
                    bus.$emit('moving-forward');
                }).catch((error) => {
                    bus.$emit('communication-error', {
                        error,
                    });
                });
            }
        },
        rotateLeft() : void {
            if (!this.isLeft) {
                this.$http.put('/robot/rotator/left', {
                    radians: 45.0 * (Math.PI / 180),
                }).then((response) => {
                    //
                }).catch((error) => {
                    bus.$emit('communication-error', {
                        error,
                    });
                });
            }
        },
        left() : void {
            if (!this.isLeft) {
                this.$http.put('/robot/motors/direction', {
                    direction: LEFT,
                }).then((response) => {
                    bus.$emit('moving-left');
                }).catch((error) => {
                    bus.$emit('communication-error', {
                        error,
                    });
                });
            }
        },
        rotateRight() : void {
            if (!this.isLeft) {
                this.$http.put('/robot/rotator/right', {
                    radians: 45.0 * (Math.PI / 180),
                }).then((response) => {
                    //
                }).catch((error) => {
                    bus.$emit('communication-error', {
                        error,
                    });
                });
            }
        },
        right() : void {
            if (!this.isRight) {
                this.$http.put('/robot/motors/direction', {
                    direction: RIGHT,
                }).then((response) => {
                    bus.$emit('moving-right');
                }).catch((error) => {
                    bus.$emit('communication-error', {
                        error,
                    });
                });
            }
        },
        reverse() : void {
            if (!this.isReverse) {
                this.$http.put('/robot/motors/direction', {
                    direction: REVERSE,
                }).then((response) => {
                    bus.$emit('moving-reverse');
                }).catch((error) => {
                    bus.$emit('communication-error', {
                        error,
                    });
                });
            }
        },
        stop() : void {
            if (this.isMoving) {
                this.$http.delete('/robot/motors').then((response) => {
                    bus.$emit('stopped');
                }).catch((error) => {
                    bus.$emit('communication-error', {
                        error,
                    });
                });
            }
        },
    },
});
</script>
