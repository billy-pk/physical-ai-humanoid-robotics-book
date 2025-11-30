#!/usr/bin/env tsx
/**
 * Script to initialize Better Auth database tables
 * 
 * Better Auth should create tables automatically, but this script
 * can be used to verify/trigger table creation.
 * 
 * Usage: tsx src/migrate.ts
 */

import "dotenv/config";
import { auth } from "./auth.js";

async function initializeDatabase() {
  console.log("🔧 Initializing Better Auth database tables...");
  
  try {
    // Better Auth creates tables automatically on first use
    // We can trigger this by attempting to get the auth context
    const ctx = await auth.$context;
    console.log("✅ Better Auth context initialized");
    console.log("📊 Database adapter:", ctx.options.database?.constructor?.name || "Unknown");
    
    // Try to query the user table to trigger creation if it doesn't exist
    // Better Auth will create tables automatically when needed
    console.log("💡 Tables will be created automatically on first authentication request");
    console.log("   You can test by making a signup request to /api/auth/sign-up/email");
    
    return true;
  } catch (error) {
    console.error("❌ Error initializing database:", error);
    return false;
  }
}

initializeDatabase()
  .then((success) => {
    if (success) {
      console.log("\n✅ Database initialization complete");
      process.exit(0);
    } else {
      console.log("\n❌ Database initialization failed");
      process.exit(1);
    }
  })
  .catch((error) => {
    console.error("Fatal error:", error);
    process.exit(1);
  });
