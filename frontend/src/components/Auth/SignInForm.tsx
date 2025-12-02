/**
 * Sign in form component with email/password authentication.
 */

import React, { useState } from "react";
import { signIn, storeSessionToken } from "../../lib/auth";
import styles from "./Auth.module.css";

interface SignInFormProps {
  onSuccess?: () => void;
  onError?: (error: string) => void;
}

export default function SignInForm({ onSuccess, onError }: SignInFormProps) {
  const [email, setEmail] = useState("");
  const [password, setPassword] = useState("");
  const [isLoading, setIsLoading] = useState(false);

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    
    // Client-side validation before submission
    const trimmedEmail = email.trim();
    
    if (!trimmedEmail) {
      onError?.("Email is required");
      return;
    }
    
    if (!password) {
      onError?.("Password is required");
      return;
    }
    
    // Basic email format validation
    const emailRegex = /^[^\s@]+@[^\s@]+\.[^\s@]+$/;
    if (!emailRegex.test(trimmedEmail)) {
      onError?.("Please enter a valid email address");
      return;
    }
    
    setIsLoading(true);

    try {
      // Log the exact data being sent
      const signinData = {
        email: trimmedEmail,
        password,
      };
      console.log("SignIn request data:", JSON.stringify(signinData, null, 2));
      
      const result = await signIn.email(signinData);
      
      console.log("SignIn result:", JSON.stringify(result, null, 2)); // Debug log

      if (result.error) {
        console.error("SignIn error:", JSON.stringify(result.error, null, 2)); // Debug log
        const errorMsg = result.error.message || result.error.code || "Sign in failed";
        onError?.(errorMsg);
      } else {
        console.log("SignIn successful, calling onSuccess"); // Debug log
        
        // Store the session token for use with FastAPI IMMEDIATELY
        const token = result.data?.token;
        if (token) {
          storeSessionToken(token);
          console.log("Session token stored for FastAPI requests:", token.substring(0, 20) + "...");
          
          // Verify token is stored
          const verifyToken = sessionStorage.getItem('better_auth_session_token');
          if (verifyToken === token) {
            console.log("Token storage verified successfully");
          } else {
            console.error("Token storage verification failed!", { stored: verifyToken, expected: token });
          }
        } else {
          console.warn("No token in sign-in response!");
        }
        
        // Small delay to ensure token is stored before navigation
        await new Promise((resolve) => setTimeout(resolve, 100));
        
        // Call onSuccess - this will trigger navigation
        onSuccess?.();
      }
    } catch (error: any) {
      console.error("SignIn exception:", error); // Debug log
      onError?.(error.message || "An unexpected error occurred");
    } finally {
      setIsLoading(false);
    }
  };

  return (
    <form onSubmit={handleSubmit} className={styles.form}>
      <h2>Sign In</h2>
      <div className={styles.field}>
        <label htmlFor="email">Email</label>
        <input
          id="email"
          type="email"
          value={email}
          onChange={(e) => setEmail(e.target.value)}
          required
          disabled={isLoading}
        />
      </div>
      <div className={styles.field}>
        <label htmlFor="password">Password</label>
        <input
          id="password"
          type="password"
          value={password}
          onChange={(e) => setPassword(e.target.value)}
          required
          disabled={isLoading}
        />
      </div>
      <button type="submit" disabled={isLoading} className={styles.button}>
        {isLoading ? "Signing in..." : "Sign In"}
      </button>
    </form>
  );
}
