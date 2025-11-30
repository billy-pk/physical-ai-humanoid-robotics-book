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
});

export const {
  signIn,
  signUp,
  signOut,
  useSession,
} = authClient;
