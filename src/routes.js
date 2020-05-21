export default [
    {
        name: 'manual',
        path: '/',
        component: require('./pages/ManualControl.vue').default,
    },
    {
        name: 'autonomous',
        path: '/autonomous',
        component: require('./pages/Autonomous.vue').default,
    },
];