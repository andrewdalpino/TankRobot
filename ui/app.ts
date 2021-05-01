import Vue from 'vue';
import VueRouter from 'vue-router';
import VueAxios from 'vue-axios';
import VueSSE from 'vue-sse';
import App from './App.vue';
import MainNav from './components/MainNav.vue';
import DirectionPad from './components/DirectionPad.vue';
import FeaturePanel from './components/FeaturePanel.vue';
import GaugeLevel from './components/GaugeLevel.vue';
import MotorThrottle from './components/MotorThrottle.vue';
import BatteryUndervoltage from  './components/BatteryUndervoltage.vue';
import CommunicationError from  './components/CommunicationError.vue';
import RolloverDetected from './components/RolloverDetected.vue';
import PageLoader from './components/PageLoader.vue';
import { Workbox } from 'workbox-window';
import routes from './routes';
import bus from  './bus';

require('./scss/app.scss');

/**
 * Register the service worker.
 */

if ('serviceWorker' in navigator) {
    const wb = new Workbox('/sw.js');

    wb.addEventListener('waiting', (event) => {
        if (event.isUpdate) {
            bus.$emit('update-ready');
        }
    });

    bus.$on('update-accepted', () => {
        wb.addEventListener('controlling', () => {
            window.location.reload();
        });
        
        wb.messageSkipWaiting();
    });
  
    wb.register();
}

/**
 * Set up the Axios HTTP client for communication with the REST API.
 */

const axios = require('axios');

/**
 * Register the Vue components and instantiate the Vue app.
 */

Vue.component('app', App);
Vue.component('main-nav', MainNav);
Vue.component('direction-pad', DirectionPad);
Vue.component('feature-panel', FeaturePanel);
Vue.component('gauge-level', GaugeLevel);
Vue.component('motor-throttle', MotorThrottle);
Vue.component('battery-undervoltage', BatteryUndervoltage);
Vue.component('communication-error', CommunicationError);
Vue.component('rollover-detected', RolloverDetected);
Vue.component('page-loader', PageLoader);

Vue.use(VueRouter);
Vue.use(VueAxios, axios);
Vue.use(VueSSE);

const router = new VueRouter({
    mode: 'history',
    base: '/ui/',
    routes,
});

const app = new Vue({
    el: '#app',
    router,
});
