import React, { createContext, useState, useEffect, useContext, ReactNode } from 'react';
import { PersonalizationPreferences } from '../types/personalization';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import ExecutionEnvironment from '@docusaurus/ExecutionEnvironment';
import { getSessionToken } from '../lib/auth';

interface PersonalizationContextType {
  preferences: PersonalizationPreferences | null;
  isLoading: boolean;
  error: string | null;
  fetchPreferences: () => Promise<void>;
  updatePreferences: (newPreferences: PersonalizationPreferences) => Promise<PersonalizationPreferences>;
}

const PersonalizationContext = createContext<PersonalizationContextType | undefined>(undefined);

interface PersonalizationProviderProps {
  children: ReactNode;
}

export const PersonalizationProvider: React.FC<PersonalizationProviderProps> = ({ children }) => {
  const [preferences, setPreferences] = useState<PersonalizationPreferences | null>(null);
  const [isLoading, setIsLoading] = useState<boolean>(true);
  const [error, setError] = useState<string | null>(null);
  const { siteConfig } = useDocusaurusContext();
  const backendUrl = siteConfig.customFields?.backendUrl as string;

  const fetchPreferences = async () => {
    if (!ExecutionEnvironment.canUseDOM) {
      return;
    }
    setIsLoading(true);
    setError(null);
    try {
      // Get session token from Better Auth to authenticate with FastAPI
      const sessionToken = await getSessionToken();
      console.log('FetchPreferences - Session token:', sessionToken ? `${sessionToken.substring(0, 20)}...` : 'null');
      console.log('FetchPreferences - Full token from sessionStorage:', sessionStorage.getItem('better_auth_session_token')?.substring(0, 20) + '...');
      
      const headers: HeadersInit = {
        'Content-Type': 'application/json',
        'Accept': 'application/json',
      };
      
      // Include session token in Authorization header if available
      if (sessionToken) {
        headers['Authorization'] = `Bearer ${sessionToken}`;
        console.log('FetchPreferences - Authorization header set with token:', sessionToken.substring(0, 20) + '...');
      } else {
        console.warn('FetchPreferences - No session token available');
        // Try to get it directly from sessionStorage as fallback
        const directToken = sessionStorage.getItem('better_auth_session_token');
        if (directToken) {
          headers['Authorization'] = `Bearer ${directToken}`;
          console.log('FetchPreferences - Using token directly from sessionStorage:', directToken.substring(0, 20) + '...');
        }
      }
      
      console.log('FetchPreferences - Making request to:', `${backendUrl}/api/auth/profile/preferences`);
      console.log('FetchPreferences - Headers:', Object.keys(headers));
      const response = await fetch(`${backendUrl}/api/auth/profile/preferences`, {
        credentials: 'include', // Include cookies for authentication
        headers,
      });
      console.log('FetchPreferences - Response status:', response.status);
      if (!response.ok) {
        if (response.status === 404) {
          setPreferences(null); // No preferences set yet
          return;
        }
        if (response.status === 401) {
          // User not authenticated, clear preferences
          setPreferences(null);
          return;
        }
        throw new Error(`Failed to fetch preferences: ${response.statusText}`);
      }
      const data: PersonalizationPreferences = await response.json();
      setPreferences(data);
    } catch (err: any) {
      console.error('Error fetching personalization preferences:', err);
      setError(err.message || 'Failed to fetch preferences.');
      setPreferences(null);
    } finally {
      setIsLoading(false);
    }
  };

  const updatePreferences = async (newPreferences: PersonalizationPreferences) => {
    if (!ExecutionEnvironment.canUseDOM) {
      return;
    }
    setIsLoading(true);
    setError(null);
    try {
      // Get session token from Better Auth to authenticate with FastAPI
      const sessionToken = await getSessionToken();
      
      const headers: HeadersInit = {
        'Content-Type': 'application/json',
        'Accept': 'application/json',
      };
      
      // Include session token in Authorization header if available
      if (sessionToken) {
        headers['Authorization'] = `Bearer ${sessionToken}`;
      }
      
      const response = await fetch(`${backendUrl}/api/auth/profile/preferences`, {
        method: 'POST',
        credentials: 'include', // Include cookies for authentication
        headers,
        body: JSON.stringify(newPreferences),
      });
      if (!response.ok) {
        const errorData = await response.json().catch(() => ({ detail: response.statusText }));
        throw new Error(errorData.detail || `Failed to update preferences: ${response.statusText}`);
      }
      const data: PersonalizationPreferences = await response.json();
      setPreferences(data);
      return data; // Return updated preferences
    } catch (err: any) {
      console.error('Error updating personalization preferences:', err);
      setError(err.message || 'Failed to update preferences.');
      throw err; // Re-throw to allow component to handle
    } finally {
      setIsLoading(false);
    }
  };

  useEffect(() => {
    // Add a delay to ensure token is stored after sign-in
    // Increased delay to ensure token is available
    const timer = setTimeout(() => {
      console.log('PersonalizationContext - Fetching preferences on mount, token check:', sessionStorage.getItem('better_auth_session_token')?.substring(0, 20) + '...');
      fetchPreferences();
    }, 300); // Increased delay to ensure token is stored
    return () => clearTimeout(timer);
  }, []); // Fetch on mount

  return (
    <PersonalizationContext.Provider value={{ preferences, isLoading, error, fetchPreferences, updatePreferences }}>
      {children}
    </PersonalizationContext.Provider>
  );
};

export const usePersonalization = () => {
  const context = useContext(PersonalizationContext);
  if (context === undefined) {
    throw new Error('usePersonalization must be used within a PersonalizationProvider');
  }
  return context;
};
