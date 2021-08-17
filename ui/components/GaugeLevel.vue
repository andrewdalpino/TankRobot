<template>
    <div class="level is-mobile">
        <div class="level-item has-text-centered">
            <div>
                <p class="heading is-size-7">{{ temperatureFahrenheit.toFixed(1) }}°</p>
                <span class="icon is-large"><i class="fas fa-2x" :class="temperatureIndicator"></i></span>
            </div>
        </div>
        <div class="level-item has-text-centered">
            <div>
                <p class="heading is-size-7">{{ sensors.battery.voltage.toFixed(1) }}V</p>
                <span class="icon is-large"><i class="fas fa-2x fa-bolt"></i></span>
            </div>
        </div>
        <div class="level-item has-text-centered">
            <div>
                <p class="heading is-size-7">{{ sensors.battery.percentage.toFixed(0) }}%</p>
                <span class="icon is-large"><i class="fas fa-2x" :class="batteryIndicator"></i></span>
            </div>
        </div>
    </div>
</template>

<script lang="ts">
import Vue from 'vue';
    
export default Vue.extend({
    props: {
        sensors: {
            type: Object,
            required: true,
        },
    },
    computed: {
        temperatureFahrenheit() : number {
            return 1.8 * this.sensors.temperature + 32.0;
        },
        temperatureIndicator() : string {
            if (this.sensors.temperature < 0.0) {
                return 'fa-temperature-low';
            } else if (this.sensors.temperature > 65.0) {
                return 'fa-temperature-high';
            } else {
                return 'fa-thermometer-half';
            }
        },
        batteryIndicator() : string {
            const p = this.sensors.battery.percentage;

            if (p <= 5) {
                return 'fa-battery-empty';
            } else if (p > 5 && p <= 25) {
                return 'fa-battery-quarter';
            } else if (p > 25 && p <= 50) {
                return 'fa-battery-half';
            } else if (p > 50 && p <= 75) {
                return 'fa-battery-three-quarters';
            } else {
                return 'fa-battery-full';
            }
        },
    },
});
</script>
