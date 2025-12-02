import React, { useState, useEffect } from 'react';
import Layout from '@theme/Layout';
import PreferenceForm from './PreferenceForm'; // Assuming PreferenceForm is in the same directory
import { usePersonalization } from '../../contexts/PersonalizationContext';
import { PersonalizationPreferences } from '../../types/personalization';

const PreferenceManagement: React.FC = () => {
  const { preferences, isLoading, error, fetchPreferences, updatePreferences } = usePersonalization();
  const [currentPreferences, setCurrentPreferences] = useState<PersonalizationPreferences | undefined>(undefined);
  const [submitError, setSubmitError] = useState<string | null>(null);
  const [isSubmitting, setIsSubmitting] = useState<boolean>(false);
  const [submitSuccess, setSubmitSuccess] = useState<boolean>(false);

  useEffect(() => {
    if (preferences) {
      setCurrentPreferences(preferences);
    }
  }, [preferences]);

  const handleSubmit = async (updatedPreferences: PersonalizationPreferences) => {
    setIsSubmitting(true);
    setSubmitError(null);
    setSubmitSuccess(false);
    try {
      await updatePreferences(updatedPreferences);
      setSubmitSuccess(true);
      // Re-fetch preferences to ensure UI is up-to-date, though updatePreferences already updates context state
      fetchPreferences();
    } catch (err: any) {
      setSubmitError(err.message || 'Failed to update preferences.');
    } finally {
      setIsSubmitting(false);
    }
  };

  if (isLoading) {
    return (
      <Layout title="Loading Preferences" description="Loading user preferences.">
        <main style={{ padding: '2rem' }}>
          <div>Loading preferences...</div>
        </main>
      </Layout>
    );
  }

  if (error && !currentPreferences) { // Only show error if no preferences could be loaded
    return (
      <Layout title="Error" description="Error loading user preferences.">
        <main style={{ padding: '2rem' }}>
          <div style={{ color: 'red' }}>Error: {error}</div>
        </main>
      </Layout>
    );
  }

  return (
    <Layout title="Manage Preferences" description="Manage your personalization preferences.">
      <main>
        <div style={{ display: 'flex', justifyContent: 'center', alignItems: 'center', padding: '2rem' }}>
          <div>
            <h1>Manage Your Preferences</h1>
            {submitSuccess && <p style={{ color: 'green' }}>Preferences updated successfully!</p>}
            <PreferenceForm
              initialPreferences={currentPreferences}
              onSubmit={handleSubmit}
              isLoading={isSubmitting}
              error={submitError || error} // Prioritize submission error, then general load error
            />
          </div>
        </div>
      </main>
    </Layout>
  );
};

export default PreferenceManagement;
