<template>
    <div>
        <main-nav></main-nav>
        <main>
            <router-view></router-view>
        </main>
        <main-footer></main-footer>
        <battery-undervoltage></battery-undervoltage>
        <rollover-detected></rollover-detected>
        <communication-error></communication-error>
        <audio id="plucky" src="./sounds/plucky.ogg" crossOrigin="anonymous"></audio>
    </div>
</template>

<script lang="ts">
import Vue from 'vue'
import bus from './providers/bus';

export default Vue.extend({
    mounted() {
        this.$sse('/events/robot/malfunctions', { format: 'json' }).then((sse) => {
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
    } 
});
</script>
