import React, { useEffect, useState } from 'react';
import { useHistory, useLocation } from '@docusaurus/router';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import { PersonalizationPreferences } from '../../types/personalization';
import ExecutionEnvironment from '@docusaurus/ExecutionEnvironment';
import { usePersonalization } from '../../contexts/PersonalizationContext';
import { getSessionToken } from '../../lib/auth';

interface PersonalizationGuardProps {
  children: React.ReactNode;
}

const PersonalizationGuard: React.FC<PersonalizationGuardProps> = ({ children }) => {
  const history = useHistory();
  const location = useLocation();
  const [hasCheckedPreferences, setHasCheckedPreferences] = useState(false);
  const { siteConfig } = useDocusaurusContext();
  const backendUrl = siteConfig.customFields?.backendUrl as string;
  const { preferences, isLoading: isContextLoading } = usePersonalization();


  useEffect(() => {
    if (!ExecutionEnvironment.canUseDOM) {
      return;
    }

    const checkPreferences = async () => {
      // Skip preference check if we're already on the popup page
      const currentPath = location.pathname;
      if (currentPath.includes('/popup')) {
        setHasCheckedPreferences(true);
        return;
      }

      // Skip preference check if preferences were just saved (to prevent redirect loop)
      if (sessionStorage.getItem('preferencesJustSaved') === 'true') {
        setHasCheckedPreferences(true);
        return;
      }

      // Skip preference check for public pages (signup, signin, home, settings)
      // Handle baseUrl by checking if path ends with public path or contains it
      const publicPaths = ['/signup', '/signin', '/settings'];
      // Check if we're on home page - handle baseUrl by checking if path ends with just baseUrl or baseUrl/
      const baseUrl = siteConfig.baseUrl || '';
      const pathWithoutBase = baseUrl ? currentPath.replace(baseUrl, '') : currentPath;
      const isHomePage = pathWithoutBase === '/' || pathWithoutBase === '' || currentPath.endsWith('/') || currentPath.match(/^\/[^\/]*\/?$/);
      const isDocsPage = currentPath.includes('/docs');
      const isPublicPath = isHomePage || isDocsPage || publicPaths.some(path => currentPath.includes(path));
      
      if (isPublicPath) {
        setHasCheckedPreferences(true);
        return;
      }

      // Wait for context to finish loading if it's still loading
      if (isContextLoading) {
        // Don't set hasCheckedPreferences yet, wait for context to load
        return;
      }

      // If context has preferences, use them instead of making another API call
      if (preferences && preferences.experience_level && preferences.content_mode) {
        setHasCheckedPreferences(true);
        return;
      }

      // For now, allow access without authentication check
      // The backend will return 401 if not authenticated, which we handle below
      // This allows users to access pages even if not logged in

      if (!backendUrl) {
        console.warn('Backend URL not configured, skipping preference check');
        setHasCheckedPreferences(true); // Allow access if backend not configured
        return;
      }

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
          method: 'GET',
          credentials: 'include', // Include cookies for authentication
          headers,
          mode: 'cors', // Explicitly set CORS mode
        });

        if (!response.ok) {
          // If 401, user is not authenticated - allow access (they can browse without preferences)
          if (response.status === 401) {
            console.log('User not authenticated, allowing access without preferences');
            setHasCheckedPreferences(true);
            return;
          }
          // If 404, it means preferences not found - redirect to popup only if authenticated
          // (If not authenticated, 401 would have been returned, so 404 means authenticated but no prefs)
          if (response.status === 404) {
            console.log('Preferences not found, redirecting to popup');
            // Only redirect if not already on popup (to avoid loops)
            if (!window.location.pathname.includes('/popup')) {
              history.push('/popup');
            } else {
              setHasCheckedPreferences(true);
            }
            return;
          }
          // For other errors, try to get error message
          const contentType = response.headers.get('content-type') || '';
          if (contentType.includes('application/json')) {
            const errorData = await response.json();
            throw new Error(errorData.detail || `Failed to fetch preferences: ${response.statusText}`);
          } else {
            const text = await response.text();
            console.error('Backend returned non-JSON response:', text.substring(0, 200));
            throw new Error(`Backend returned non-JSON response: ${response.statusText}`);
          }
        }
        
        // Check if response is actually JSON before parsing
        const contentType = response.headers.get('content-type') || '';
        if (!contentType.includes('application/json')) {
          const text = await response.text();
          console.error('Backend returned non-JSON response:', text.substring(0, 200));
          setHasCheckedPreferences(true);
          return;
        }
        
        const fetchedPreferences: PersonalizationPreferences = await response.json();
        
        // Check if essential preferences (e.g., experience_level) are set
        if (!fetchedPreferences.experience_level || !fetchedPreferences.content_mode) {
          // Only redirect if not already on popup and not on a public path
          if (!currentPath.includes('/popup') && !isPublicPath) {
            history.push('/popup');
          } else {
            setHasCheckedPreferences(true);
          }
        } else {
          setHasCheckedPreferences(true);
        }
      } catch (err: any) {
        console.error('Error checking user preferences:', err);
        // Check if it's a CORS error
        if (err.message && (err.message.includes('CORS') || err.message.includes('Failed to fetch'))) {
          console.warn('CORS error detected. Make sure the backend is running and CORS is configured correctly.');
          // Allow access if CORS fails (for development)
          setHasCheckedPreferences(true);
          return;
        }
        // On other errors, only redirect if not on a public path
        // If on public path, allow access
        if (!isPublicPath && !currentPath.includes('/popup')) {
          history.push('/popup');
        } else {
          setHasCheckedPreferences(true);
        }
      }
    };

    checkPreferences();
  }, [history, backendUrl, location.pathname, preferences, isContextLoading, siteConfig.baseUrl]); 

  if (!hasCheckedPreferences) {
    return <div>Loading preferences...</div>; 
  }

  return <>{children}</>;
};

export default PersonalizationGuard;