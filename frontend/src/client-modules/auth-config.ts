/**
 * Client module to inject Better Auth URL into window object
 * This runs before the app initializes
 */

export function onRouteDidUpdate() {
  // This function is called after every route change
  if (typeof window !== 'undefined' && !(window as any).__BETTER_AUTH_URL__) {
    // Set the Better Auth service URL for development
    (window as any).__BETTER_AUTH_URL__ =
      process.env.NODE_ENV === 'production'
        ? 'https://your-auth-service.com' // Replace with your production auth service URL
        : 'http://localhost:3001';

    console.log('[Auth Config] Better Auth URL set to:', (window as any).__BETTER_AUTH_URL__);
  }
}
