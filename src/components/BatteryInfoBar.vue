<template>
    <div class="level is-mobile">
        <div class="level-item has-text-centered">
            <div>
                <p class="heading is-size-7">{{ battery_voltage.toFixed(1) }} V</p>
                <span class="icon is-large">
                    <i class="fas fa-2x fa-bolt"></i>
                </span>
            </div>
            <div>
                <p class="heading is-size-7">{{ battery_percentage }} %</p>
                <span class="icon is-large">
                    <i class="fas fa-2x" :class="battery_indicator"></i>
                </span>
            </div>
        </div>
    </div>
</template>

<script>
    const MAX_VOLTAGE = 8.4;

    export default {
        props: {
            tank: {
                type: Object,
                required: true,
            },
        },
        computed: {
            battery_voltage() {
                return this.tank.battery_level * MAX_VOLTAGE;
            },
            battery_percentage() {
                return Math.round(this.tank.battery_level * 100.0);
            },
            battery_indicator() {
                const level = this.tank.battery_level;

                if (level <= 0.05) {
                    return 'fa-battery-empty';
                } else if (level > 0.05 && level <= 0.25) {
                    return 'fa-battery-quarter';
                } else if (level > 0.25 && level <= 0.5) {
                    return 'fa-battery-half';
                } else if (level > 0.5 && level <= 0.75) {
                    return 'fa-battery-three-quarters';
                } else {
                    return 'fa-battery-full';
                }
            },
        },
    }
</script>