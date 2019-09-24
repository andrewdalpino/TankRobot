<template>
    <div class="level is-mobile">
        <div class="level-item has-text-centered">
            <div>
                <p class="heading is-size-7">{{ tank.battery_voltage.toFixed(1) }} V</p>
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
    const MIN_VOLTAGE = 6.0;
    const MAX_VOLTAGE = 8.4;

    export default {
        props: {
            tank: {
                type: Object,
                required: true,
            },
        },
        computed: {
            battery_percentage() {
                return Math.round(((this.tank.battery_voltage - MIN_VOLTAGE) / (MAX_VOLTAGE - MIN_VOLTAGE) * 100.0));
            },
            battery_indicator() {
                const p = this.tank.battery_percentage;

                if (p <= 5.0) {
                    return 'fa-battery-empty';
                } else if (p > 5.0 && p <= 25.0) {
                    return 'fa-battery-quarter';
                } else if (p > 25.0 && p <= 50.0) {
                    return 'fa-battery-half';
                } else if (p > 50.0 && p <= 75.0) {
                    return 'fa-battery-three-quarters';
                } else {
                    return 'fa-battery-full';
                }
            },
        },
    }
</script>