<template>
    <nav class="navbar is-white is-fixed-top" role="navigation" aria-label="main navigation">
        <div class="navbar-brand">
            <span class="navbar-item is-size-4 is-uppercase">Kirby The Tank Robot</span>
            <a role="button" class="navbar-burger burger" aria-label="menu" aria-expanded="false" data-target="nav-menu" @click="open = !open">
                <span aria-hidden="true"></span>
                <span aria-hidden="true"></span>
                <span aria-hidden="true"></span>
            </a>
        </div>
        <div id="nav-menu" class="navbar-menu" :class="{ 'is-active' : open }">
            <div class="navbar-end">
                <div class="navbar-item">
                    <router-link :to="{ name: 'basic'}" class="button is-white is-fullwidth" @click.native="open = false">
                        <span class="icon"><i class="fas fa-arrows-alt"></i></span>
                        <span>Basic Controls</span>
                    </router-link>
                </div>
            </div>
        </div>
    </nav>
</template>

<script>
    export default {
        data() {
            return {
                open: false,
            };
        },
    }
</script>
