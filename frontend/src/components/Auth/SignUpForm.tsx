/**
 * Sign up form component with email/password authentication.
 */

import React, { useState } from "react";
import { signUp, storeSessionToken } from "../../lib/auth";
import styles from "./Auth.module.css";

interface SignUpFormProps {
  onSuccess?: () => void;
  onError?: (error: string) => void;
}

export default function SignUpForm({ onSuccess, onError }: SignUpFormProps) {
  const [email, setEmail] = useState("");
  const [password, setPassword] = useState("");
  const [name, setName] = useState("");
  const [isLoading, setIsLoading] = useState(false);

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    
    // Client-side validation before submission
    const trimmedEmail = email.trim();
    const trimmedName = name.trim();
    
    if (!trimmedEmail) {
      onError?.("Email is required");
      return;
    }
    
    if (!trimmedName) {
      onError?.("Name is required");
      return;
    }
    
    if (!password) {
      onError?.("Password is required");
      return;
    }
    
    if (password.length < 8) {
      onError?.("Password must be at least 8 characters long");
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
      const signupData = {
        email: trimmedEmail,
        password,
        name: trimmedName,
      };
      console.log("SignUp request data:", JSON.stringify(signupData, null, 2));
      
      const result = await signUp.email(signupData);

      console.log("SignUp result:", JSON.stringify(result, null, 2)); // Debug log

      if (result.error) {
        console.error("SignUp error:", JSON.stringify(result.error, null, 2)); // Debug log
        // Handle specific error codes
        if (result.error.code === "USER_ALREADY_EXISTS_USE_ANOTHER_EMAIL" || 
            result.error.code === "USER_ALREADY_EXISTS" ||
            result.error.message?.includes("already exists") ||
            result.error.message?.includes("already registered")) {
          onError?.("This email is already registered. Please sign in instead or use a different email.");
        } else {
          const errorMsg = result.error.message || result.error.code || JSON.stringify(result.error);
          console.error("SignUp error details:", errorMsg);
          onError?.(errorMsg || "Sign up failed. Please try again.");
        }
      } else {
        console.log("SignUp successful, calling onSuccess"); // Debug log
        
        // Store the session token for use with FastAPI IMMEDIATELY
        const token = result.data?.token;
        if (token) {
          storeSessionToken(token);
          console.log("Session token stored for FastAPI requests:", token.substring(0, 20) + "...");
        } else {
          console.warn("No token in sign-up response!");
        }
        
        // Call onSuccess immediately - don't wait
        // The token is already stored, so preferences fetch can use it
        onSuccess?.();
      }
    } catch (error: any) {
      console.error("SignUp exception:", error); // Debug log
      onError?.(error.message || "An unexpected error occurred");
    } finally {
      setIsLoading(false);
    }
  };

  return (
    <form onSubmit={handleSubmit} className={styles.form}>
      <h2>Sign Up</h2>
      <div className={styles.field}>
        <label htmlFor="name">Name</label>
        <input
          id="name"
          type="text"
          value={name}
          onChange={(e) => setName(e.target.value)}
          required
          disabled={isLoading}
        />
      </div>
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
          minLength={8}
          disabled={isLoading}
        />
        <small>Minimum 8 characters</small>
      </div>
      <button type="submit" disabled={isLoading} className={styles.button}>
        {isLoading ? "Signing up..." : "Sign Up"}
      </button>
    </form>
  );
}
