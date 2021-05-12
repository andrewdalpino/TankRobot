<template>
    <nav class="navbar is-fixed-top" role="navigation" aria-label="main navigation">
        <div class="navbar-brand">
            <h1 class="navbar-item">Tank Robot</h1>
            <a role="button" class="navbar-burger burger" aria-label="menu" aria-expanded="false" @click="open = !open">
                <span aria-hidden="true"></span>
                <span aria-hidden="true"></span>
                <span aria-hidden="true"></span>
            </a>
        </div>
        <div class="navbar-menu" :class="{ 'is-active' : open }">
            <div class="navbar-end">
                <div class="navbar-item">
                    <router-link :to="{ name: 'control' }" class="navbar-item" @click.native="open = false">
                        <span class="icon"><i class="fas fa-arrows-alt"></i></span>
                        <span>Controls</span>
                    </router-link>
                </div>
                <div class="navbar-item">
                    <router-link :to="{ name: 'autonomy' }" class="navbar-item" @click.native="open = false">
                        <span class="icon"><i class="fas fa-brain"></i></span>
                        <span>Autonomy</span>
                    </router-link>
                </div>
            </div>
        </div>
    </nav>
</template>

<script lang="ts">
import Vue from 'vue';

export default Vue.extend({
    data() {
        return {
            open: false,
        };
    },
});
</script>
