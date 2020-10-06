<template>
    <section class="section">
        <information-panel :tank="tank"></information-panel>
        <div class="columns is-mobile">
            <div class="column is-one-quarter">
                <throttle :tank="tank"></throttle>
            </div>
            <div class="column is-three-quarters">
                Nothing yet
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
