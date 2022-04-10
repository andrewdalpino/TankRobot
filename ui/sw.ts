import { precacheAndRoute, createHandlerBoundToURL, cleanupOutdatedCaches } from 'workbox-precaching';
import { registerRoute, NavigationRoute } from 'workbox-routing';
import precacheManifest from './precache-manifest';

/**
 * Precache static assets.
 */

precacheAndRoute(precacheManifest);

/**
 * Serve the app shell for navigation requests.
 */

const handler = createHandlerBoundToURL('/ui');

const navigationRoute = new NavigationRoute(handler, {
    allowlist: [
        new RegExp('/ui/'),
      ],
});

registerRoute(navigationRoute);

/**
 * Listen for skip waiting signal from main thread.
 */

addEventListener('message', (event) => {
    if (event.data && event.data.type === 'SKIP_WAITING') {
        skipWaiting();
    }
});

/**
 * Clean up caches used by previous versions.
 */

cleanupOutdatedCaches();
