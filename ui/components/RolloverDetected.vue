<template>
    <div class="modal" :class="{ 'is-active' : open }">
        <div class="modal-background"></div>
        <div class="modal-content">
            <div class="box has-text-centered">
                <div class="block">
                    <span class="icon is-large">
                        <i class="fas fa-2x fa-car-crash"></i>
                    </span>
                </div>
                <p class="block is-size-5">
                    A rollover was detected. Check the orientation of the robot.
                </p>
                <div class="block buttons is-centered">
                    <button class="button is-danger is-outlined" @click="open = false">
                        <span class="icon"><i class="fas fa-times"></i></span>
                        <span>Dismiss</span>
                    </button>
                </div>
            </div>
        </div>
    </div>
</template>

<script lang="ts">
import Vue from 'vue';
import bus from '../providers/bus';

const VIBRATE_PATTERN = [100, 30, 100];

export default Vue.extend({
    data() {
        return {
            sound: null,
            open: false,
        };
    },
    methods: {
        handleRolloverDetected() : void {
            if (!this.open) {
                this.open = true;

                this.beep();
                this.vibrate();
            }
        },
        beep() : void {
            if (this.sound) {
                this.sound.play();
            }
        },
        vibrate() : void {
            window.navigator.vibrate(VIBRATE_PATTERN);
        },
    },
    mounted() {
        const element = document.getElementById('plucky');

        if (element instanceof HTMLAudioElement) {
            this.sound = element;
        }

        bus.$on('rollover-detected', this.handleRolloverDetected);
    },
});
</script>
