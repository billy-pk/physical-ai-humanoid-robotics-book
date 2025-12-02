import React, { useEffect, useState } from 'react';
import Layout from '@theme/Layout';
import { useLocation } from '@docusaurus/router';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import { usePersonalization } from '../contexts/PersonalizationContext';
import FullView from '../components/Content/FullView'; // Assuming this path
import ContentModeSwitch from '../components/Content/ContentModeSwitch'; // Import ContentModeSwitch
import TranslationToggle from '../components/Content/TranslationToggle'; // Import TranslationToggle
import { PersonalizationPreferences } from '../types/personalization';


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
      const response = await fetch(`${backendUrl}/api/content/chapters/${chapter_id}`);
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
      const response = await fetch(`${backendUrl}/api/content/translate`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
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
    if (!preferences) return;
    const updatedPreferences = { ...preferences, urdu_translation_enabled: enabled };
    try {
      await updatePreferences(updatedPreferences);
      if (enabled && originalChapterContent) {
        await fetchTranslatedContent(originalChapterContent);
        setDisplayingTranslated(true);
      } else {
        setDisplayingTranslated(false);
      }
    } catch (err) {
      console.error("Failed to update translation preference:", err);
    }
  };


  const currentContent = displayingTranslated ? translatedChapterContent : originalChapterContent;

  if (isLoadingContent || isPreferencesLoading) {
    return (
      <Layout title="Loading..." description="Loading chapter content.">
        <main style={{ padding: '2rem' }}>
          <div>Loading chapter...</div>
        </main>
      </Layout>
    );
  }

  if (contentError) {
    return (
      <Layout title="Error" description="Error loading chapter content.">
        <main style={{ padding: '2rem' }}>
          <div style={{ color: 'red' }}>Error: {contentError}</div>
        </main>
      </Layout>
    );
  }

  if (!currentContent) {
    return (
      <Layout title="Not Found" description="Chapter content not found.">
        <main style={{ padding: '2rem' }}>
          <div>Chapter content not found.</div>
        </main>
      </Layout>
    );
  }

  return (
    <Layout title={chapter_id} description={`Chapter: ${chapter_id}`}>
      <main>
        <div style={{ padding: '2rem' }}>
          <div style={{ display: 'flex', justifyContent: 'space-between', marginBottom: '1rem' }}>
            {preferences && (
              <ContentModeSwitch
                currentMode={preferences.content_mode}
                onModeChange={handleModeChange}
                isLoading={isPreferencesLoading}
              />
            )}
            {preferences && (
                <TranslationToggle
                    urduTranslationEnabled={preferences.urdu_translation_enabled}
                    onToggle={handleTranslationToggle}
                    isLoading={isPreferencesLoading || isTranslating}
                />
            )}
          </div>
          <h1>{chapter_id}</h1>
          <FullView content={currentContent} />
        </div>
      </main>
    </Layout>
  );
};

export default ChapterPage;