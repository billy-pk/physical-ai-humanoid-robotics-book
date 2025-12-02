import React from 'react';
import { PersonalizationPreferences } from '../../types/personalization';

interface ContentModeSwitchProps {
  currentMode: PersonalizationPreferences['content_mode'];
  onModeChange: (newMode: PersonalizationPreferences['content_mode']) => void;
  isLoading: boolean;
}

const ContentModeSwitch: React.FC<ContentModeSwitchProps> = ({
  currentMode,
  onModeChange,
  isLoading,
}) => {
  return (
    <div className="content-mode-switch">
      <label htmlFor="contentModeToggle">Content Mode:</label>
      <select
        id="contentModeToggle"
        value={currentMode}
        onChange={(e) => onModeChange(e.target.value as PersonalizationPreferences['content_mode'])}
        disabled={isLoading}
      >
        <option value="full">Full Content</option>
        <option value="personalized">Personalized Content</option>
      </select>
    </div>
  );
};

export default ContentModeSwitch;
