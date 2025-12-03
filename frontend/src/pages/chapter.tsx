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


  const fetchChapterContent = async () => {
    setIsLoadingContent(true);
    setContentError(null);
    setOriginalChapterContent(null);
    setTranslatedChapterContent(null);

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
      setOriginalChapterContent(textContent);
      
      // If urduTranslationEnabled, default to displaying translated content
      // and fetch translated content if not already fetched or if original content changed
      if (preferences?.urdu_translation_enabled) {
        if (!translatedChapterContent || textContent !== originalChapterContent) { // Fetch if translated content is stale or non-existent
          await fetchTranslatedContent(textContent); 
        }
        setDisplayingTranslated(true);
      } else {
        setDisplayingTranslated(false);
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
      fetchChapterContent();
    }
  }, [chapter_id, preferences, isPreferencesLoading, preferencesError, backendUrl]);

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
      // If preferences exist, update them
      if (preferences) {
        const updatedPreferences = { ...preferences, urdu_translation_enabled: enabled };
        await updatePreferences(updatedPreferences);
      }
      
      // Always allow translation toggle to work, even if preferences aren't loaded
      if (enabled && originalChapterContent) {
        await fetchTranslatedContent(originalChapterContent);
        setDisplayingTranslated(true);
      } else {
        setDisplayingTranslated(false);
      }
    } catch (err) {
      console.error("Failed to update translation preference:", err);
      // Still allow toggling display even if preference update fails
      if (enabled && originalChapterContent) {
        await fetchTranslatedContent(originalChapterContent);
        setDisplayingTranslated(true);
      } else {
        setDisplayingTranslated(false);
      }
    }
  };


  const currentContent = displayingTranslated ? translatedChapterContent : originalChapterContent;


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