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
    <div 
      className={`translation-toggle ${urduTranslationEnabled ? 'translation-toggle--active' : ''} ${isLoading ? 'translation-toggle--loading' : ''}`}
      style={{ 
        padding: '10px 16px',
        borderRadius: '8px',
        display: 'inline-flex',
        alignItems: 'center',
        transition: 'all 0.2s ease',
        opacity: isLoading ? 0.7 : 1,
      }}
    >
      <label style={{ 
        display: 'flex', 
        alignItems: 'center', 
        gap: '10px', 
        cursor: isLoading ? 'not-allowed' : 'pointer',
        margin: 0,
        fontSize: '14px',
        fontWeight: '600',
        userSelect: 'none'
      }}>
        <input
          type="checkbox"
          checked={urduTranslationEnabled}
          onChange={(e) => onToggle(e.target.checked)}
          disabled={isLoading}
          style={{ 
            width: '20px', 
            height: '20px', 
            cursor: isLoading ? 'not-allowed' : 'pointer',
            margin: 0,
          }}
        />
        <span>
          Translate to Urdu
        </span>
      </label>
    </div>
  );
};

export default TranslationToggle;
