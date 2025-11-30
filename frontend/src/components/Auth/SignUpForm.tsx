/**
 * Sign up form component with email/password authentication.
 */

import React, { useState } from "react";
import { signUp } from "../../lib/auth";
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
    setIsLoading(true);

    try {
      const result = await signUp.email({
        email,
        password,
        name,
      });

      console.log("SignUp result:", result); // Debug log

      if (result.error) {
        console.error("SignUp error:", result.error); // Debug log
        // Handle specific error codes
        if (result.error.code === "USER_ALREADY_EXISTS_USE_ANOTHER_EMAIL") {
          onError?.("This email is already registered. Please sign in instead or use a different email.");
        } else {
          onError?.(result.error.message || "Sign up failed");
        }
      } else {
        console.log("SignUp successful, calling onSuccess"); // Debug log
        // Wait a moment for session to be established
        await new Promise((resolve) => setTimeout(resolve, 500));
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
