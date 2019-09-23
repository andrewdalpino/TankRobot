<template>
    <section class="section">
        <h1 class="title">Manual Controls</h1>
        <direction-pad :tank="tank"></direction-pad>
        <throttle :tank="tank"></throttle>
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
