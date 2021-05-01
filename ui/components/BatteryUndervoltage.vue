<template>
    <div class="modal" :class="{ 'is-active' : open }">
        <div class="modal-background"></div>
        <div class="modal-content">
            <div class="box has-text-centered">
                <div class="block">
                    <span class="icon is-large">
                        <i class="fas fa-2x fa-battery-empty"></i>
                    </span>
                </div>
                <p class="block is-size-5">
                    Battery Undervoltage Detected
                </p>
                <p class="block help">
                    Battery voltage of {{ voltage }}V is too low. Recharge or replace the cells.
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
import bus from  '../bus';

const VIBRATE_PATTERN = [100, 30, 100];

export default Vue.extend({
    data() {
        return {
            sound: null,
            open: false,
            voltage: null,
        };
    },
    mounted() {
        const element = document.getElementById('plucky');

        if (element instanceof HTMLAudioElement) {
            this.sound = element;
        }

        bus.$on('battery-undervoltage', (event) => {
            if (!this.open) {
                this.voltage = event.voltage;
                this.open = true;

                this.beep();
                this.vibrate();
            }
        });
    },
    methods: {
        beep() : void {
            if (this.sound) {
                this.sound.play();
            }
        },
        vibrate() : void {
            window.navigator.vibrate(VIBRATE_PATTERN);
        },
    },
});
</script>
