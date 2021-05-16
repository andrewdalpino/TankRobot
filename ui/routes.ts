import Control from './pages/Control.vue';
import Autonomy from './pages/Autonomy.vue';
import Training from './pages/Training.vue';

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
    {
        name: 'autonomy',
        path: '/autonomy',
        component: Autonomy,
    },
    {
        name: 'training',
        path: '/training',
        component: Training,
    },
];
