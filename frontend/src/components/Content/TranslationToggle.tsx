import React from 'react';

interface TranslationToggleProps {
  urduTranslationEnabled: boolean;
  onToggle: (enabled: boolean) => void;
  isLoading: boolean;
}

const TranslationToggle: React.FC<TranslationToggleProps> = ({
  urduTranslationEnabled,
  onToggle,
  isLoading,
}) => {
  return (
    <div className="translation-toggle">
      <label>
        <input
          type="checkbox"
          checked={urduTranslationEnabled}
          onChange={(e) => onToggle(e.target.checked)}
          disabled={isLoading}
        />
        Translate to Urdu
      </label>
    </div>
  );
};

export default TranslationToggle;
