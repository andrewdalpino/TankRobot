<template>
    <section class="section">
        <battery-info-bar :tank="tank"></battery-info-bar>
        <div class="columns is-mobile">
            <div class="column is-one-quarter">
                <throttle :tank="tank"></throttle>
            </div>
            <div class="column is-three-quarters">
                <direction-pad :tank="tank"></direction-pad>
            </div>
        </div>
    </section>
</template>

<script>
    export default {
        props: {
            tank: {
                type: Object,
                required: true,
            },
        },
    }
</script>
