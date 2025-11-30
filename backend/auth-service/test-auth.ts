#!/usr/bin/env tsx
/**
 * Test script for Better Auth service
 * 
 * Tests signup and signin endpoints to verify Better Auth is working correctly.
 * 
 * Usage:
 *   tsx test-auth.ts
 */

import { config } from "./src/config.js";

const AUTH_URL = config.auth.baseURL;

async function testSignup() {
  console.log("\n🧪 Testing signup endpoint...");
  
  const response = await fetch(`${AUTH_URL}/api/auth/sign-up/email`, {
    method: "POST",
    headers: {
      "Content-Type": "application/json",
    },
    body: JSON.stringify({
      email: `test-${Date.now()}@example.com`,
      password: "testpassword123",
      name: "Test User",
    }),
  });

  const data = await response.json();
  
  if (response.ok) {
    console.log("✅ Signup successful!");
    console.log("   User ID:", data.user?.id);
    console.log("   Email:", data.user?.email);
    return data;
  } else {
    console.log("❌ Signup failed:", data);
    throw new Error(`Signup failed: ${JSON.stringify(data)}`);
  }
}

async function testSignin(email: string, password: string) {
  console.log("\n🧪 Testing signin endpoint...");
  
  const response = await fetch(`${AUTH_URL}/api/auth/sign-in/email`, {
    method: "POST",
    headers: {
      "Content-Type": "application/json",
    },
    body: JSON.stringify({
      email,
      password,
    }),
  });

  const data = await response.json();
  
  if (response.ok) {
    console.log("✅ Signin successful!");
    console.log("   User ID:", data.user?.id);
    console.log("   Session:", data.session ? "Created" : "None");
    return data;
  } else {
    console.log("❌ Signin failed:", data);
    throw new Error(`Signin failed: ${JSON.stringify(data)}`);
  }
}

async function testSession(cookies: string[]) {
  console.log("\n🧪 Testing session endpoint...");
  
  const response = await fetch(`${AUTH_URL}/api/auth/session`, {
    method: "GET",
    headers: {
      "Cookie": cookies.join("; "),
    },
  });

  const data = await response.json();
  
  if (response.ok && data.user) {
    console.log("✅ Session valid!");
    console.log("   User ID:", data.user.id);
    console.log("   Email:", data.user.email);
    return data;
  } else {
    console.log("❌ Session invalid or expired");
    return null;
  }
}

async function testHealth() {
  console.log("\n🧪 Testing health endpoint...");
  
  const response = await fetch(`${AUTH_URL}/health`);
  const data = await response.json();
  
  if (response.ok && data.status === "ok") {
    console.log("✅ Health check passed!");
    return true;
  } else {
    console.log("❌ Health check failed:", data);
    return false;
  }
}

async function main() {
  console.log("🚀 Starting Better Auth tests...");
  console.log(`📍 Auth URL: ${AUTH_URL}`);
  
  try {
    // Test health endpoint
    const healthOk = await testHealth();
    if (!healthOk) {
      console.log("\n❌ Health check failed. Is the server running?");
      process.exit(1);
    }
    
    // Test signup
    const signupResult = await testSignup();
    const testEmail = signupResult.user.email;
    const testPassword = "testpassword123";
    
    // Extract cookies from signup response (if any)
    // Note: In a real browser, cookies would be set automatically
    console.log("\n💡 Note: Cookies are not automatically handled in this test script.");
    console.log("   In a real browser, cookies would be set automatically.");
    
    // Test signin
    await testSignin(testEmail, testPassword);
    
    console.log("\n✅ All tests passed!");
    console.log("\n📝 Next steps:");
    console.log("   1. Start the server: npm run dev");
    console.log("   2. Test in browser or with Postman/curl");
    console.log("   3. Verify Better Auth created user/session tables in database");
    
  } catch (error) {
    console.error("\n❌ Test failed:", error);
    process.exit(1);
  }
}

main();
