<template>
    <div class="buttons">
        <button class="button is-large" @click="beep()">
            <span class="icon"><i class="fas fa-bullhorn"></i></span>
        </button>
        <button class="button is-large" :class="{ 'is-success' : features.autonomy }" @click="toggleAutonomy()">
            <span class="icon"><i class="fas fa-brain"></i></span>
        </button>
    </div>
</template>

<script lang="ts">
import Vue from  'vue';
import bus from '../bus';

export default Vue.extend({
    props: {
        features: {
            type: Object,
            required: true,
        }
    },
    methods: {
        beep() : void {
            this.$http.put('/robot/features/beeper').catch((error) => {
                bus.$emit('communication-error', {
                    error,
                });
            });
        },
        toggleAutonomy() : void {
            if (this.features.autonomy) {
                this.$http.delete('/robot/features/autonomy').then(() => {
                    bus.$emit('autonomy-disabled');
                }).catch((error) => {
                    bus.$emit('communication-error', {
                        error,
                    });
                });
            } else {
                this.$http.put('/robot/features/autonomy').then(() => {
                    bus.$emit('autonomy-enabled');
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
