/**
 * Better Auth configuration and instance
 * 
 * This file configures Better Auth with:
 * - PostgreSQL database connection (Neon Postgres)
 * - Email/password authentication
 * - Session management
 */

import { betterAuth } from "better-auth";
import { Pool } from "pg";
import { config } from "./config.js";

// Create PostgreSQL connection pool
const pool = new Pool({
  connectionString: config.database.url,
  ssl: {
    rejectUnauthorized: false, // Required for Neon Postgres
  },
});

// Configure Better Auth instance
export const auth = betterAuth({
  database: pool,
  baseURL: config.auth.baseURL,
  secret: config.auth.secret,
  trustedOrigins: config.auth.trustedOrigins,
  
  // Email and password authentication
  emailAndPassword: {
    enabled: true,
    requireEmailVerification: false, // Can enable later if needed
    minPasswordLength: 8,
    maxPasswordLength: 128,
    autoSignIn: true, // Automatically sign in after signup
  },

  // Session configuration
  session: {
    expiresIn: 60 * 60 * 24 * 7, // 7 days
    updateAge: 60 * 60 * 24, // Update session every 24 hours
    cookieCache: {
      enabled: true,
      maxAge: 5 * 60, // Cache for 5 minutes
    },
  },

  // Cookie configuration for cross-port compatibility in development
  advanced: {
    cookieOptions: {
      sameSite: "lax", // Allow cookies across ports on localhost
      httpOnly: true,
      secure: false, // Set to true in production with HTTPS
      path: "/",
    },
  },
});
