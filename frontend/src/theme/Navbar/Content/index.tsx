import React, {type ReactNode, useState, useEffect} from 'react';
import Content from '@theme-original/Navbar/Content';
import type ContentType from '@theme/Navbar/Content';
import type {WrapperProps} from '@docusaurus/types';
import { usePersonalization } from '../../../contexts/PersonalizationContext';
import TranslationToggle from '../../../components/Content/TranslationToggle';
import {useLocation} from '@docusaurus/router';

type Props = WrapperProps<typeof ContentType>;

export default function ContentWrapper(props: Props): ReactNode {
  const location = useLocation(); // Track location to force updates
  
  let preferences, isLoading, updatePreferences;
  
  try {
    const personalization = usePersonalization();
    preferences = personalization.preferences;
    isLoading = personalization.isLoading;
    updatePreferences = personalization.updatePreferences;
  } catch (err) {
    console.error('[Navbar] PersonalizationContext not available:', err);
    preferences = null;
    isLoading = false;
    updatePreferences = async () => {};
  }

  const handleTranslationToggle = async (enabled: boolean) => {
    try {
      if (preferences && updatePreferences) {
        const updatedPreferences = { ...preferences, urdu_translation_enabled: enabled };
        await updatePreferences(updatedPreferences);
      }
    } catch (err) {
      console.error("Failed to update translation preference:", err);
    }
  };

  // Always render the toggle - navbar should persist across navigation
  return (
    <>
      <Content {...props} />
      <div className="navbar-translation-toggle-item">
        <TranslationToggle
          urduTranslationEnabled={preferences?.urdu_translation_enabled || false}
          onToggle={handleTranslationToggle}
          isLoading={isLoading || false}
        />
      </div>
    </>
  );
}
