import Control from './pages/Control.vue';

export default [
    {
        name: 'home',
        path: '/',
        redirect: { name: 'control' },
    },
    {
        name: 'control',
        path: '/control',
        component: Control,
    },
];