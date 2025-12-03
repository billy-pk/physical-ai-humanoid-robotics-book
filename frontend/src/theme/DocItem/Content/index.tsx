import React, { useEffect, useState } from 'react';
import Content from '@theme-original/DocItem/Content';
import type ContentType from '@theme/DocItem/Content';
import type {WrapperProps} from '@docusaurus/types';
import { useDoc } from '@docusaurus/plugin-content-docs/client';
import { usePersonalization } from '../../../contexts/PersonalizationContext';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Markdown from 'react-markdown';
import remarkGfm from 'remark-gfm';
import { getSessionToken } from '../../../lib/auth';
import { useLocation } from '@docusaurus/router';

type Props = WrapperProps<typeof ContentType>;

export default function ContentWrapper(props: Props): React.ReactElement {
  const doc = useDoc();
  const location = useLocation(); // Track location changes
  const { preferences, isLoading: isPreferencesLoading } = usePersonalization();
  const { siteConfig } = useDocusaurusContext();
  const backendUrl = siteConfig.customFields?.backendUrl as string;

  const [translatedContent, setTranslatedContent] = useState<string | null>(null);
  const [isLoadingTranslation, setIsLoadingTranslation] = useState(false);
  const [translationError, setTranslationError] = useState<string | null>(null);

  // Extract chapter_id from doc metadata or path
  const getChapterId = (): string | null => {
    try {
      // Try multiple properties to find chapter_id
      // 1. Try frontMatter.id
      if (doc?.frontMatter?.id) {
        return doc.frontMatter.id;
      }

      // 2. Try metadata.id (this is the one that works)
      if (doc?.metadata?.id) {
        return doc.metadata.id.replace(/^\/+/, '');
      }

      // 3. Try metadata.unversionedId
      if (doc?.metadata?.unversionedId) {
        return doc.metadata.unversionedId.replace(/^\/+/, '');
      }

      // 4. Try doc.id
      if (doc?.id) {
        return doc.id.replace(/^\/+/, '');
      }

      // 5. Extract from location pathname as last resort
      const pathname = location.pathname;
      const match = pathname.match(/\/docs\/(.+?)(?:\/|$)/);
      if (match && match[1]) {
        const chapterId = match[1].replace(/\/$/, '');
        return chapterId;
      }

      console.warn('[DocItem] Could not determine chapter_id from any source');
      return null;
    } catch (err) {
      console.error('[DocItem] Error getting chapter_id:', err);
      return null;
    }
  };

  useEffect(() => {
    const fetchTranslatedContent = async () => {
      // Only fetch if Urdu translation is enabled
      if (!preferences?.urdu_translation_enabled) {
        setTranslatedContent(null);
        return;
      }

      const chapterId = getChapterId();
      if (!chapterId) {
        console.warn('[DocItem] Could not determine chapter_id');
        return;
      }

      console.log('[DocItem] Fetching translated content for:', chapterId);
      setIsLoadingTranslation(true);
      setTranslationError(null);

      try {
        const sessionToken = await getSessionToken();
        const headers: HeadersInit = {
          'Content-Type': 'application/json',
          'Accept': 'text/markdown',
        };

        if (sessionToken) {
          headers['Authorization'] = `Bearer ${sessionToken}`;
        }

        const response = await fetch(`${backendUrl}/api/content/chapters/${chapterId}`, {
          headers,
          credentials: 'include',
        });

        if (!response.ok) {
          throw new Error(`Failed to fetch translated content: ${response.statusText}`);
        }

        const textContent = await response.text();
        setTranslatedContent(textContent);
      } catch (err: any) {
        console.error('[DocItem] Error fetching translated content:', err);
        setTranslationError(err.message);
        setTranslatedContent(null);
      } finally {
        setIsLoadingTranslation(false);
      }
    };

    if (!isPreferencesLoading && doc) {
      fetchTranslatedContent();
    }
  }, [preferences?.urdu_translation_enabled, doc?.id, isPreferencesLoading, backendUrl, location.pathname]); // Added location.pathname to trigger on navigation

  // Listen for preferences updates
  useEffect(() => {
    const handlePreferencesUpdated = () => {
      // The main useEffect will handle refetching due to preferences dependency
    };

    window.addEventListener('preferencesUpdated', handlePreferencesUpdated as EventListener);
    return () => {
      window.removeEventListener('preferencesUpdated', handlePreferencesUpdated as EventListener);
    };
  }, []);

  // If translation is enabled and we have translated content, render it
  if (preferences?.urdu_translation_enabled && translatedContent) {
    return (
      <div className="theme-doc-markdown markdown">
        <Markdown remarkPlugins={[remarkGfm]}>{translatedContent}</Markdown>
      </div>
    );
  }

  // If loading translation, show loading state
  if (preferences?.urdu_translation_enabled && isLoadingTranslation) {
    return (
      <div className="theme-doc-markdown markdown">
        <p>Loading translated content...</p>
      </div>
    );
  }

  // If translation failed, show error but fall back to original
  if (preferences?.urdu_translation_enabled && translationError) {
    console.warn('[DocItem] Translation error, falling back to original content:', translationError);
  }

  // Otherwise, render the original Docusaurus content
  return <Content {...props} />;
}
