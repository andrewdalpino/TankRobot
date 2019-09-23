<template>
    <section class="section">
        <h1 class="title">Basic Controls</h1>
        <direction-pad></direction-pad>
        <speed-control :tank="tank"></speed-control>
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
