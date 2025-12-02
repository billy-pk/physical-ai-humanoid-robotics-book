/**
 * Better Auth client configuration for frontend.
 * 
 * This file configures the Better Auth client to connect to the auth service.
 */

import { createAuthClient } from "better-auth/react";

// Better Auth service URL - defaults to localhost:3001 for development
// In production, this should be set via environment variable
const BETTER_AUTH_URL = 
  typeof window !== "undefined" 
    ? (window as any).__BETTER_AUTH_URL__ || "http://localhost:3001"
    : "http://localhost:3001";

export const authClient = createAuthClient({
  baseURL: BETTER_AUTH_URL,
  fetchOptions: {
    credentials: 'include', // Ensure cookies are sent with requests
  },
});

export const {
  signIn,
  signUp,
  signOut,
  useSession,
} = authClient;

/**
 * Store session token after sign-in.
 * The token from the sign-in response should be stored here.
 * This clears any old tokens and stores the new one.
 */
export function storeSessionToken(token: string): void {
  if (typeof window !== "undefined") {
    // Clear any old token first
    sessionStorage.removeItem('better_auth_session_token');
    // Store the new token
    sessionStorage.setItem('better_auth_session_token', token);
    console.log('storeSessionToken - Token stored:', token.substring(0, 20) + '...');
  }
}

/**
 * Get the current session token.
 * First checks sessionStorage (from sign-in), then tries to get it from Better Auth.
 */
export async function getSessionToken(): Promise<string | null> {
  if (typeof window === "undefined") {
    return null;
  }

  // First, check if we stored the token after sign-in
  const storedToken = sessionStorage.getItem('better_auth_session_token');
  if (storedToken) {
    console.log('getSessionToken - Using stored token from sessionStorage');
    return storedToken;
  }

  console.log('getSessionToken - No stored token, trying Better Auth get-session');
  
  // If not stored, try to get it from Better Auth's get-session endpoint
  // This will work if the cookie is set and accessible
  try {
    const response = await fetch(`${BETTER_AUTH_URL}/api/auth/get-session`, {
      credentials: 'include',
      headers: {
        'Content-Type': 'application/json',
      },
    });

    if (!response.ok) {
      console.log('getSessionToken - Better Auth get-session failed:', response.status);
      return null;
    }

    const data = await response.json();
    console.log('getSessionToken - Better Auth response:', data);
    
    // Better Auth get-session returns: { user: {...}, session: {...} }
    // The session token is typically in session.token or session.id
    if (data?.session?.token) {
      // Store it for future use
      sessionStorage.setItem('better_auth_session_token', data.session.token);
      console.log('getSessionToken - Stored token from session.token');
      return data.session.token;
    }
    
    if (data?.session?.id) {
      // Store it for future use
      sessionStorage.setItem('better_auth_session_token', data.session.id);
      console.log('getSessionToken - Stored token from session.id');
      return data.session.id;
    }
    
    console.log('getSessionToken - No token found in Better Auth response');
    return null;
  } catch (error) {
    console.error('Error getting session token:', error);
    return null;
  }
}

/**
 * Clear the stored session token (e.g., on sign-out).
 */
export function clearSessionToken(): void {
  if (typeof window !== "undefined") {
    sessionStorage.removeItem('better_auth_session_token');
  }
}
