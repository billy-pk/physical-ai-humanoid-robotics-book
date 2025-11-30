/**
 * Express server for Better Auth service
 * 
 * This server handles all authentication requests at /api/auth/*
 * Runs on port 3001 by default (configurable via PORT env var)
 */

// Load environment variables first
import "dotenv/config";

import express from "express";
import cors from "cors";
import { toNodeHandler } from "better-auth/node";
import { auth } from "./auth.js";
import { config } from "./config.js";

const app = express();

// Configure CORS middleware - must be before Better Auth handler
app.use(
  cors({
    origin: config.auth.trustedOrigins,
    methods: ["GET", "POST", "PUT", "DELETE", "OPTIONS"],
    credentials: true, // Allow cookies
    allowedHeaders: ["Content-Type", "Authorization", "Cookie"],
  })
);

// Health check endpoint
app.get("/health", (req, res) => {
  res.json({ status: "ok", service: "better-auth" });
});

// Mount Better Auth handler - handles all /api/auth/* routes
app.all("/api/auth/*", toNodeHandler(auth));

// Mount express json middleware AFTER Better Auth handler
// Better Auth handles its own body parsing, so we only apply json() to other routes
app.use(express.json());

// 404 handler for non-auth routes
app.use((req, res) => {
  res.status(404).json({ error: "Not found" });
});

// Error handler
app.use((err: Error, req: express.Request, res: express.Response, next: express.NextFunction) => {
  console.error("Error:", err);
  res.status(500).json({ error: "Internal server error" });
});

app.listen(config.server.port, () => {
  console.log(`✅ Better Auth service running on http://localhost:${config.server.port}`);
  console.log(`📡 Auth endpoints available at http://localhost:${config.server.port}/api/auth/*`);
  console.log(`💚 Health check: http://localhost:${config.server.port}/health`);
  console.log(`🔗 Base URL: ${config.auth.baseURL}`);
  console.log(`🌐 Trusted origins: ${config.auth.trustedOrigins.join(", ")}`);
});
