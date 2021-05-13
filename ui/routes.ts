import Control from './pages/Control.vue';
import Autonomy from './pages/Autonomy.vue';
import TrainingLoss from './pages/TrainingLoss.vue';

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
        name: 'training-loss',
        path: '/training-loss',
        component: TrainingLoss,
    },
];
