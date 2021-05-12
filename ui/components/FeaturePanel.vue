<template>
    <div class="buttons">
        <button class="button is-large" @click="beep()">
            <span class="icon"><i class="fas fa-bullhorn"></i></span>
        </button>
    </div>
</template>

<script lang="ts">
import Vue from  'vue';
import bus from '../providers/bus';

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
    },
});
</script>
