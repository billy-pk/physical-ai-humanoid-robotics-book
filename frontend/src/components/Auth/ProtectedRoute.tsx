/**
 * Protected Route component.
 * 
 * Wraps content that requires authentication.
 * Redirects to sign-in if user is not authenticated.
 */

import React from "react";
import { useAuth } from "../../contexts/AuthContext";
import SignInForm from "./SignInForm";

interface ProtectedRouteProps {
  children: React.ReactNode;
  fallback?: React.ReactNode;
}

export default function ProtectedRoute({ children, fallback }: ProtectedRouteProps) {
  const { user, isLoading } = useAuth();

  if (isLoading) {
    return (
      <div style={{ padding: "2rem", textAlign: "center" }}>
        <p>Loading...</p>
      </div>
    );
  }

  if (!user) {
    return (
      fallback || (
        <div style={{ maxWidth: "400px", margin: "2rem auto" }}>
          <h2>Sign In Required</h2>
          <p>Please sign in to access this content.</p>
          <SignInForm />
        </div>
      )
    );
  }

  return <>{children}</>;
}
