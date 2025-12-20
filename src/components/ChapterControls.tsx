import React, { useState } from 'react';
import { useAuth } from '../hooks/useAuth';
import styles from './ChapterControls.module.css';

interface ChapterControlsProps {
  onPersonalizationToggle: (enabled: boolean) => void;
  onTranslationToggle: (enabled: boolean) => void;
  isTranslating?: boolean;
}

export default function ChapterControls({
  onPersonalizationToggle,
  onTranslationToggle,
  isTranslating = false
}: ChapterControlsProps) {
  const { user } = useAuth();

  const [personalizationEnabled, setPersonalizationEnabled] = useState(
    user?.enablePersonalization ?? false
  );

  const [translationEnabled, setTranslationEnabled] = useState(
    user?.enableUrduTranslation ?? false
  );

  // Don't show controls if user is not logged in
  if (!user) {
    return null;
  }

  const handlePersonalizationToggle = () => {
    const newValue = !personalizationEnabled;
    setPersonalizationEnabled(newValue);
    onPersonalizationToggle(newValue);
  };

  const handleTranslationToggle = () => {
    if (isTranslating) return; // Prevent toggle during translation

    const newValue = !translationEnabled;
    setTranslationEnabled(newValue);
    onTranslationToggle(newValue);
  };

  return (
    <div className={styles.controls}>
      <div className={styles.controlGroup}>
        <button
          className={`${styles.controlButton} ${personalizationEnabled ? styles.active : ''}`}
          onClick={handlePersonalizationToggle}
          title="Toggle personalized tooltips based on your experience level"
          aria-label="Toggle personalization"
          aria-pressed={personalizationEnabled}
        >
          <svg width="20" height="20" viewBox="0 0 20 20" fill="currentColor">
            <path d="M10 2a8 8 0 100 16 8 8 0 000-16zm0 14a6 6 0 110-12 6 6 0 010 12z"/>
            <circle cx="10" cy="10" r="2"/>
          </svg>
          <span>Personalize</span>
        </button>

        <button
          className={`${styles.controlButton} ${translationEnabled ? styles.active : ''}`}
          onClick={handleTranslationToggle}
          title="Translate content to Urdu"
          aria-label="Toggle Urdu translation"
          aria-pressed={translationEnabled}
          disabled={isTranslating}
          style={{ opacity: isTranslating ? 0.6 : 1 }}
        >
          <svg width="20" height="20" viewBox="0 0 20 20" fill="currentColor">
            <path d="M10 2C5.58 2 2 5.58 2 10s3.58 8 8 8 8-3.58 8-8-3.58-8-8-8zm4.5 12H5.5v-1h9v1zm0-3H5.5V10h9v1zm0-3H5.5V7h9v1z"/>
          </svg>
          <span>اردو (Urdu)</span>
        </button>
      </div>

      {personalizationEnabled && (
        <div className={styles.statusBadge}>
          ✓ Tooltips tailored to your <strong>{user.programmingExperience}</strong> programming level,{' '}
          <strong>{user.rosFamiliarity}</strong> ROS familiarity, and{' '}
          <strong>{user.aiMlBackground}</strong> AI/ML background
        </div>
      )}

      {translationEnabled && !isTranslating && (
        <div className={styles.statusBadge}>
          ✓ Content translated to Urdu (اردو)
        </div>
      )}

      {isTranslating && (
        <div className={styles.loading}>
          <div className={styles.loadingSpinner}></div>
          <span>Translating to Urdu...</span>
        </div>
      )}
    </div>
  );
}
