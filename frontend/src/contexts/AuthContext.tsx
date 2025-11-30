/**
 * Authentication context provider for React.
 * 
 * Wraps Better Auth session management in a React context.
 */

import React, { createContext, useContext, ReactNode } from "react";
import { useSession } from "../lib/auth";

interface AuthContextType {
  user: any | null;
  session: any | null;
  isLoading: boolean;
}

const AuthContext = createContext<AuthContextType | undefined>(undefined);

export function AuthProvider({ children }: { children: ReactNode }) {
  const { data, isPending } = useSession();

  const value: AuthContextType = {
    user: data?.user || null,
    session: data?.session || null,
    isLoading: isPending,
  };

  return <AuthContext.Provider value={value}>{children}</AuthContext.Provider>;
}

export function useAuth() {
  const context = useContext(AuthContext);
  if (context === undefined) {
    throw new Error("useAuth must be used within an AuthProvider");
  }
  return context;
}
