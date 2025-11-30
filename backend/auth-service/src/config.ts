/**
 * Configuration and environment variables for Better Auth service
 */

// Load environment variables from .env file
import "dotenv/config";

export const config = {
  database: {
    url: process.env.DATABASE_URL || "",
  },
  auth: {
    secret: process.env.BETTER_AUTH_SECRET || "",
    baseURL: process.env.BETTER_AUTH_URL || "http://localhost:3001",
    trustedOrigins: process.env.BETTER_AUTH_TRUSTED_ORIGINS?.split(",").map(s => s.trim()) || [
      "http://localhost:3000",
    ],
  },
  server: {
    port: parseInt(process.env.PORT || "3001", 10),
  },
};

// Validate required environment variables
if (!config.database.url) {
  throw new Error("DATABASE_URL environment variable is required");
}

if (!config.auth.secret) {
  throw new Error("BETTER_AUTH_SECRET environment variable is required");
}

if (config.auth.secret.length < 32) {
  throw new Error("BETTER_AUTH_SECRET must be at least 32 characters long");
}
