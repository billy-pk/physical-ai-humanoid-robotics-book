import React, { useEffect, useState } from 'react';
import Layout from '@theme/Layout';
import { useLocation } from '@docusaurus/router';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import { usePersonalization } from '../contexts/PersonalizationContext';
import FullView from '../components/Content/FullView'; // Assuming this path
import ContentModeSwitch from '../components/Content/ContentModeSwitch'; // Import ContentModeSwitch
import TranslationToggle from '../components/Content/TranslationToggle'; // Import TranslationToggle
import { PersonalizationPreferences } from '../types/personalization';
import { getSessionToken } from '../lib/auth'; // Import getSessionToken


const ChapterPage: React.FC = () => {
  const location = useLocation();
  // Extract chapter_id from URL
  // Try query parameter first (e.g., ?chapter_id=module-1/chapter-3)
  // Then try path segment (e.g., /chapter/module-1/chapter-3)
  const searchParams = new URLSearchParams(location.search);
  let chapter_id = searchParams.get('chapter_id') || 
                   location.pathname.replace(/^.*\/chapter\//, '').replace(/\/$/, '') || 
                   'intro';
  const { siteConfig } = useDocusaurusContext();
  const backendUrl = siteConfig.customFields?.backendUrl as string;
  const { preferences, isLoading: isPreferencesLoading, error: preferencesError, updatePreferences } = usePersonalization();

  const [originalChapterContent, setOriginalChapterContent] = useState<string | null>(null); // Store original content fetched from API
  const [translatedChapterContent, setTranslatedChapterContent] = useState<string | null>(null); // Store translated content
  const [displayingTranslated, setDisplayingTranslated] = useState<boolean>(false); // State to toggle display
  
  const [isLoadingContent, setIsLoadingContent] = useState(true);
  const [contentError, setContentError] = useState<string | null>(null);
  const [isTranslating, setIsTranslating] = useState<boolean>(false);

  // Debug: Log when component renders
  console.log('[ChapterPage] Component render - preferences:', preferences);
  console.log('[ChapterPage] urdu_translation_enabled:', preferences?.urdu_translation_enabled);


  const fetchChapterContent = async () => {
    setIsLoadingContent(true);
    setContentError(null);
    // Don't clear existing content - we want to preserve it for toggling

    if (isPreferencesLoading) {
      // Still loading preferences, wait for next render cycle
      return;
    }
    if (preferencesError) {
      setContentError("Error loading preferences. Cannot fetch content.");
      setIsLoadingContent(false);
      return;
    }

    try {
      // Get session token for authentication
      const sessionToken = await getSessionToken();
      const headers: HeadersInit = {
        'Content-Type': 'application/json',
        'Accept': 'text/markdown',
      };
      
      if (sessionToken) {
        headers['Authorization'] = `Bearer ${sessionToken}`;
      }
      
      const response = await fetch(`${backendUrl}/api/content/chapters/${chapter_id}`, {
        headers,
        credentials: 'include',
      });
      
      if (!response.ok) {
        throw new Error(`Failed to fetch chapter content: ${response.statusText}`);
      }
      const textContent = await response.text();
      
      console.log('[ChapterPage] Fetched content, urdu_translation_enabled:', preferences?.urdu_translation_enabled);
      console.log('[ChapterPage] Content preview (first 200 chars):', textContent.substring(0, 200));
      
      // Backend already returns translated content when urdu_translation_enabled is true
      if (preferences?.urdu_translation_enabled) {
        // Backend returned translated content, store it as translated
        setTranslatedChapterContent(textContent);
        setDisplayingTranslated(true);
        console.log('[ChapterPage] Stored as translated content, displaying translated');
      } else {
        // Backend returned original content
        setOriginalChapterContent(textContent);
        setDisplayingTranslated(false);
        console.log('[ChapterPage] Stored as original content, displaying original');
      }

    } catch (err: any) {
      console.error('Error fetching chapter content:', err);
      setContentError(err.message || 'Failed to load chapter content.');
    } finally {
      setIsLoadingContent(false);
    }
  };

  const fetchTranslatedContent = async (contentToTranslate: string) => {
    setIsTranslating(true);
    setContentError(null);
    try {
      // Get session token for authentication
      const sessionToken = await getSessionToken();
      const headers: HeadersInit = {
        'Content-Type': 'application/json',
        'Accept': 'text/markdown',
      };
      
      if (sessionToken) {
        headers['Authorization'] = `Bearer ${sessionToken}`;
      }
      
      const response = await fetch(`${backendUrl}/api/content/translate`, {
        method: 'POST',
        headers,
        credentials: 'include',
        body: JSON.stringify({
          chapter_id: chapter_id,
          target_language: 'ur',
          force_regenerate: false, // Could be an option later
        }),
      });
      if (!response.ok) {
        throw new Error(`Failed to translate content: ${response.statusText}`);
      }
      const translatedText = await response.text();
      setTranslatedChapterContent(translatedText);
    } catch (err: any) {
      console.error('Error fetching translated content:', err);
      setContentError(err.message || 'Failed to load translated content.');
      setTranslatedChapterContent(null);
    } finally {
      setIsTranslating(false);
    }
  };

  useEffect(() => {
    // Only fetch chapter content once preferences are loaded
    if (!isPreferencesLoading) {
      console.log('[ChapterPage] useEffect triggered - fetching content, urdu_translation_enabled:', preferences?.urdu_translation_enabled);
      fetchChapterContent();
    }
  }, [chapter_id, preferences?.urdu_translation_enabled, preferences?.content_mode, isPreferencesLoading, preferencesError, backendUrl]);

  // Listen for preferences update events (from navbar toggle)
  useEffect(() => {
    const handlePreferencesUpdated = (e: Event) => {
      console.log('[ChapterPage] Preferences updated event received!', e);
      console.log('[ChapterPage] Event detail:', (e as CustomEvent).detail);
      console.log('[ChapterPage] Refetching content...');
      // Always refetch when preferences are updated
      fetchChapterContent();
    };

    console.log('[ChapterPage] Setting up preferencesUpdated event listener');
    window.addEventListener('preferencesUpdated', handlePreferencesUpdated as EventListener);
    console.log('[ChapterPage] Event listener added to window');

    return () => {
      console.log('[ChapterPage] Cleaning up preferencesUpdated event listener');
      window.removeEventListener('preferencesUpdated', handlePreferencesUpdated as EventListener);
    };
  }, []);

  const handleModeChange = async (newMode: PersonalizationPreferences['content_mode']) => {
    if (!preferences) return;
    const updatedPreferences = { ...preferences, content_mode: newMode };
    try {
      await updatePreferences(updatedPreferences);
      // Re-fetch content to apply new mode if needed, or rely on preferences update in context
      fetchChapterContent();
    } catch (err) {
      console.error("Failed to update content mode:", err);
    }
  };

  const handleTranslationToggle = async (enabled: boolean) => {
    try {
      // Update preferences - this will trigger useEffect to refetch content
      if (preferences) {
        const updatedPreferences = { ...preferences, urdu_translation_enabled: enabled };
        await updatePreferences(updatedPreferences);
        // Refetch content immediately - backend will return translated/original based on preference
        await fetchChapterContent();
      }
    } catch (err) {
      console.error("Failed to update translation preference:", err);
    }
  };


  const currentContent = displayingTranslated ? translatedChapterContent : originalChapterContent;
  
  // Debug logging
  useEffect(() => {
    console.log('[ChapterPage] Display state:', {
      displayingTranslated,
      hasTranslated: !!translatedChapterContent,
      hasOriginal: !!originalChapterContent,
      currentContentLength: currentContent?.length || 0,
      urduTranslationEnabled: preferences?.urdu_translation_enabled
    });
  }, [displayingTranslated, translatedChapterContent, originalChapterContent, currentContent, preferences]);


  return (
    <Layout title={chapter_id} description={`Chapter: ${chapter_id}`}>
      <main>
        <div style={{ padding: '2rem' }}>
          {preferences && (
            <div style={{ 
              display: 'flex', 
              justifyContent: 'flex-start', 
              marginBottom: '1rem', 
              alignItems: 'center',
              width: '100%',
              minHeight: '50px',
              padding: '10px',
              backgroundColor: 'var(--ifm-background-surface-color)',
              borderRadius: '8px',
              border: '1px solid var(--ifm-color-emphasis-300)'
            }}>
              <ContentModeSwitch
                currentMode={preferences.content_mode}
                onModeChange={handleModeChange}
                isLoading={isPreferencesLoading}
              />
            </div>
          )}
          
          <h1>{chapter_id}</h1>
          
          {isLoadingContent || isPreferencesLoading ? (
            <div>Loading chapter...</div>
          ) : contentError ? (
            <div style={{ color: 'red' }}>Error: {contentError}</div>
          ) : !currentContent ? (
            <div>Chapter content not found.</div>
          ) : (
            <FullView content={currentContent} />
          )}
        </div>
      </main>
    </Layout>
  );
};

export default ChapterPage;