<template>
    <section class="section">
        <information-panel :tank="tank"></information-panel>
        <div class="columns is-mobile">
            <div class="column is-one-quarter">
                <throttle :tank="tank"></throttle>
            </div>
            <div class="column is-three-quarters">
                <direction-pad :tank="tank"></direction-pad>
            </div>
        </div>
        <stability-assist :tank="tank"></stability-assist>
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
