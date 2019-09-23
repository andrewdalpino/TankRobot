import Vue from 'vue';
import VueRouter from 'vue-router'
import VueAxios from 'vue-axios';
import routes from './routes';

require('./scss/app.scss');

/**
 * Register the background service worker.
 */

if ('serviceWorker' in navigator) {
    window.addEventListener('load', () => {
        navigator.serviceWorker.register('/sw.js');
    });
}

/**
 * Set up the Axios HTTP client for communication with the backend API.
 */

const axios = require('axios');

axios.defaults.baseURL = 'http://192.168.4.1/api';
axios.defaults.timeout = 1500;

/**
 * The following block of code may be used to automatically register your
 * Vue components. It will recursively scan this directory for the Vue
 * components and automatically register them with their "basename".
 *
 * Eg. ./components/ExampleComponent.vue -> <example-component></example-component>
 */

const files = require.context('./', true, /\.vue$/i);
files.keys().map(key => Vue.component(key.split('/').pop().split('.')[0], files(key).default));

/**
 * Next, we will create a fresh Vue application instance and attach it to
 * the page. Then, you may begin adding components to this application
 * or customize the JavaScript scaffolding to fit your unique needs.
 */

Vue.use(VueRouter);

Vue.use(VueAxios, axios);

const router = new VueRouter({
    mode: 'history',
    routes,
});

const app = new Vue({
    el: '#app',
    router,
});