/**
 * Sign in form component with email/password authentication.
 */

import React, { useState } from "react";
import { signIn } from "../../lib/auth";
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
    setIsLoading(true);

    try {
      const result = await signIn.email({
        email,
        password,
      });

      if (result.error) {
        onError?.(result.error.message || "Sign in failed");
      } else {
        onSuccess?.();
      }
    } catch (error: any) {
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
