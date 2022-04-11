<template>
    <div class="level is-mobile">
        <div class="level-item has-text-centered">
            <div>
                <p class="heading is-size-7">{{ visibilityPercentage.toFixed(0) }}%</p>
                <span class="icon is-large"><i class="fas fa-2x" :class="visibilityIndicator"></i></span>
            </div>
        </div>
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
                <p class="heading is-size-7">{{ batteryPercentage.toFixed(0) }}%</p>
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
        visibilityPercentage() : number {
            return 100 * this.sensors.lidar.visibility;
        },
        visibilityIndicator() : string {
            const visibility = this.sensors.lidar.visibility;

            if (visibility > 0.5) {
                return 'fa-eye';
            } else {
                return 'fa-eye-low-vision';
            }
        },
        temperatureFahrenheit() : number {
            return 1.8 * this.sensors.temperature + 32.0;
        },
        temperatureIndicator() : string {
            const temperature = this.sensors.temperature;

            if (temperature < 0.0) {
                return 'fa-temperature-low';
            } else if (temperature > 65.0) {
                return 'fa-temperature-high';
            } else {
                return 'fa-thermometer-half';
            }
        },
        batteryPercentage() : number {
            return 100 * this.sensors.battery.capacity;
        },
        batteryIndicator() : string {
            const capacity = this.sensors.battery.capacity;

            if (capacity > 0.8) {
                return 'fa-battery-full';
            } else if (capacity > 0.6) {
                return 'fa-battery-three-quarters';
            } else if (capacity > 0.4) {
                return 'fa-battery-half';
            } else if (capacity > 0.2) {
                return 'fa-battery-quarter';
            } else {
                return 'fa-battery-empty';
            }
        },
    },
});
</script>
