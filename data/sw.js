self.addEventListener('install', function(event) {
    event.waitUntil(caches.open('precache').then(function(cache) {
        return cache.addAll([
            '/',
            '/app.js',
            '/app.css',
            '/fa-solid-900.woff',
            '/fa-solid-900.woff2',
            '/app-icon-large.png',
            '/app-icon-small.png',
            '/plucky.ogg',
            '/manifest.json',
        ]);
    }));
});

self.addEventListener('fetch', function(event) {
    event.respondWith(caches.match(event.request).then(function(response) {
        return response || fetch(event.request);
    }));
});