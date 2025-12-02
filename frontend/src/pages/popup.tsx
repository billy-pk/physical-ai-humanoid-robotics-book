import React, { useState } from 'react';
import Layout from '@theme/Layout';
import PreferenceForm from '../components/Auth/PreferenceForm';
import { PersonalizationPreferences } from '../types/personalization';
import { useHistory } from '@docusaurus/router';
import { usePersonalization } from '../contexts/PersonalizationContext';

const Popup: React.FC = () => {
  const history = useHistory();
  const { updatePreferences, fetchPreferences, isLoading: isContextLoading } = usePersonalization();
  const [isSubmitting, setIsSubmitting] = useState(false);
  const [error, setError] = useState<string | null>(null);

  const handleSubmit = async (preferences: PersonalizationPreferences) => {
    setIsSubmitting(true);
    setError(null);
    try {
      await updatePreferences(preferences);
      console.log('Preferences saved successfully');
      
      // Set a flag to indicate preferences were just saved
      // This prevents PersonalizationGuard from redirecting back immediately
      sessionStorage.setItem('preferencesJustSaved', 'true');
      
      // Refresh preferences in context to ensure they're up to date
      await fetchPreferences();
      
      // Small delay to ensure state is updated
      await new Promise(resolve => setTimeout(resolve, 300));
      
      // Redirect to home page
      history.push('/');
      
      // Clear the flag after a short delay
      setTimeout(() => {
        sessionStorage.removeItem('preferencesJustSaved');
      }, 1000);
    } catch (err: any) {
      setError(err.message || 'Failed to save preferences.');
    } finally {
      setIsSubmitting(false);
    }
  };

  return (
    <Layout title="Preferences" description="User personalization preferences">
      <main>
        <div style={{ display: 'flex', justifyContent: 'center', alignItems: 'center', padding: '2rem' }}>
          <PreferenceForm
            onSubmit={handleSubmit}
            isLoading={isSubmitting || isContextLoading}
            error={error}
          />
        </div>
      </main>
    </Layout>
  );
};

export default Popup;
