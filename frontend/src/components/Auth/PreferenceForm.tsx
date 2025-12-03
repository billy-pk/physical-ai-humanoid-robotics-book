import React, { useState } from 'react';
import { PersonalizationPreferences } from '../../types/personalization';

interface PreferenceFormProps {
  initialPreferences?: PersonalizationPreferences;
  onSubmit: (preferences: PersonalizationPreferences) => void;
  isLoading: boolean;
  error: string | null;
}

const PreferenceForm: React.FC<PreferenceFormProps> = ({
  initialPreferences,
  onSubmit,
  isLoading,
  error,
}) => {
  const [experienceLevel, setExperienceLevel] = useState(
    initialPreferences?.experience_level || '' // Changed to empty string for validation
  );
  const [learningTopics, setLearningTopics] = useState<string[]>(
    initialPreferences?.learning_topics || []
  );
  const [learningGoals, setLearningGoals] = useState(
    initialPreferences?.learning_goals || ''
  );
  const [contentMode, setContentMode] = useState(
    initialPreferences?.content_mode || '' // Changed to empty string for validation
  );
  const [validationErrors, setValidationErrors] = useState<Record<string, string | null>>({});

  const validateForm = () => {
    const errors: Record<string, string | null> = {};
    let isValid = true;

    if (!experienceLevel) {
      errors.experienceLevel = 'Experience Level is required.';
      isValid = false;
    }

    if (learningTopics.length === 0) {
      errors.learningTopics = 'At least one learning topic is required.';
      isValid = false;
    } else if (learningTopics.length > 10) {
      errors.learningTopics = 'Maximum 10 learning topics allowed.';
      isValid = false;
    } else if (learningTopics.some(topic => topic.length === 0 || topic.length > 50)) {
      errors.learningTopics = 'Each topic must be between 1 and 50 characters.';
      isValid = false;
    }

    if (!learningGoals.trim()) {
      errors.learningGoals = 'Learning Goals are required.';
      isValid = false;
    } else if (learningGoals.length > 500) {
      errors.learningGoals = 'Learning Goals cannot exceed 500 characters.';
      isValid = false;
    }

    if (!contentMode) {
      errors.contentMode = 'Content Mode is required.';
      isValid = false;
    }

    setValidationErrors(errors);
    return isValid;
  };

  const handleSubmit = (e: React.FormEvent) => {
    e.preventDefault();
    if (!validateForm()) {
      return;
    }

    const preferences: PersonalizationPreferences = {
      experience_level: experienceLevel as 'beginner' | 'intermediate' | 'advanced',
      learning_topics: learningTopics,
      learning_goals: learningGoals,
      content_mode: contentMode as 'full' | 'personalized',
      urdu_translation_enabled: initialPreferences?.urdu_translation_enabled || false, // Keep existing value, don't allow editing in form
      // These will be set by the backend
      preferences_submitted_at: initialPreferences?.preferences_submitted_at,
      preferences_last_updated_at: initialPreferences?.preferences_last_updated_at,
      preferences_version: initialPreferences?.preferences_version || 1,
    };
    onSubmit(preferences);
  };

  return (
    <form onSubmit={handleSubmit} className="preference-form">
      <h2>Your Learning Preferences</h2>
      {error && <p className="error-message">{error}</p>}

      <div className="form-group">
        <label htmlFor="experienceLevel">Experience Level:</label>
        <select
          id="experienceLevel"
          value={experienceLevel}
          onChange={(e) => {setExperienceLevel(e.target.value); setValidationErrors(prev => ({...prev, experienceLevel: null}))}}
          disabled={isLoading}
        >
          <option value="">Select Level</option> {/* Added a placeholder option */}
          <option value="beginner">Beginner</option>
          <option value="intermediate">Intermediate</option>
          <option value="advanced">Advanced</option>
        </select>
        {validationErrors.experienceLevel && <p className="error-text">{validationErrors.experienceLevel}</p>}
      </div>

      <div className="form-group">
        <label htmlFor="learningTopics">Learning Topics (comma-separated):</label>
        <input
          id="learningTopics"
          type="text"
          value={learningTopics.join(', ')}
          onChange={(e) => {
            setLearningTopics(e.target.value.split(',').map(topic => topic.trim()).filter(topic => topic !== ''));
            setValidationErrors(prev => ({...prev, learningTopics: null}));
          }}
          disabled={isLoading}
          placeholder="e.g., ROS 2, Computer Vision, Path Planning"
        />
        {validationErrors.learningTopics && <p className="error-text">{validationErrors.learningTopics}</p>}
      </div>

      <div className="form-group">
        <label htmlFor="learningGoals">Learning Goals (max 500 chars):</label>
        <textarea
          id="learningGoals"
          value={learningGoals}
          onChange={(e) => {setLearningGoals(e.target.value); setValidationErrors(prev => ({...prev, learningGoals: null}))}}
          maxLength={500}
          disabled={isLoading}
          rows={4}
          placeholder="Describe what you hope to achieve (e.g., build a SLAM robot, understand motion planning algorithms)"
        />
        {validationErrors.learningGoals && <p className="error-text">{validationErrors.learningGoals}</p>}
      </div>

      <div className="form-group">
        <label htmlFor="contentMode">Content Mode:</label>
        <select
          id="contentMode"
          value={contentMode}
          onChange={(e) => {setContentMode(e.target.value); setValidationErrors(prev => ({...prev, contentMode: null}))}}
          disabled={isLoading}
        >
          <option value="">Select Mode</option> {/* Added a placeholder option */}
          <option value="full">Full Content (unmodified)</option>
          <option value="personalized">Personalized Content (AI-adapted)</option>
        </select>
        {validationErrors.contentMode && <p className="error-text">{validationErrors.contentMode}</p>}
      </div>

      <button type="submit" disabled={isLoading}>
        {isLoading ? 'Saving...' : 'Save Preferences'}
      </button>
    </form>
  );
};

export default PreferenceForm;
