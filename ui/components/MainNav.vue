<template>
    <nav class="navbar is-fixed-top" role="navigation" aria-label="main navigation">
        <div class="navbar-brand">
            <span class="navbar-item">
                <img src="../images/app-icon-small.png" width="28" height="28" loading="eager" alt="Tank Robot logo" />
            </span>
            <h1 class="navbar-item">Tank Robot</h1>
            <a role="button" class="navbar-burger burger" :class="{ 'is-active': open }" aria-label="menu" aria-expanded="false" @click="open = !open">
                <span aria-hidden="true"></span>
                <span aria-hidden="true"></span>
                <span aria-hidden="true"></span>
            </a>
        </div>
        <div class="navbar-menu" :class="{ 'is-active' : open }">
            <div class="navbar-start">
                <div class="navbar-item">
                    <router-link :to="{ name: 'control' }" class="navbar-item" @click.native="open = false">
                        <span class="icon"><i class="fas fa-arrows-alt"></i></span>
                        <span>Control</span>
                    </router-link>
                </div>
                <div class="navbar-item">
                    <router-link :to="{ name: 'dynamics' }" class="navbar-item" @click.native="open = false">
                        <span class="icon"><i class="fas fa-road"></i></span>
                        <span>Dynamics</span>
                    </router-link>
                </div>
                <div class="navbar-item">
                    <router-link :to="{ name: 'autonomy' }" class="navbar-item" @click.native="open = false">
                        <span class="icon"><i class="fas fa-brain"></i></span>
                        <span>Autonomy</span>
                    </router-link>
                </div>
                <div class="navbar-item">
                    <router-link :to="{ name: 'training' }" class="navbar-item" @click.native="open = false">
                        <span class="icon"><i class="fas fa-chart-line"></i></span>
                        <span>Training</span>
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
