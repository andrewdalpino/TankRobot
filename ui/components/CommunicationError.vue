<template>
    <div class="modal" :class="{ 'is-active' : open }">
        <div class="modal-background"></div>
        <div class="modal-content">
            <div class="box has-text-centered">
                <div class="block">
                    <span class="icon is-large">
                        <span class="fa-stack fa-lg">
                            <i class="fas fa-wifi fa-stack-1x"></i>
                            <i class="fas fa-ban fa-stack-2x has-text-danger"></i>
                        </span>
                    </span>
                </div>
                <p class="block is-size-5">
                    Communication Error
                </p>
                <p class="block help">
                    {{ message }}
                </p>
                <div class="block buttons is-centered">
                    <button class="button is-info is-outlined" @click="$router.go()">
                        <span class="icon"><i class="fas fa-redo-alt"></i></span>
                        <span>Retry</span>
                    </button>
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
            message: 'Unknown error.',
        };
    },
    methods: {
        handleCommunicationError() : void {
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

        bus.$on('communication-error', this.handleCommunicationError);
    },
});
</script>
