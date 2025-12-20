import React, { useState, useEffect, useRef } from 'react';
import { useAuth } from '../hooks/useAuth';
import { usePersonalizationSafe } from '../context/PersonalizationContext';
import { TRANSLATION_API_URL } from '../config/api';
import styles from './FloatingToolbar.module.css';

interface FloatingToolbarProps {
  onClose?: () => void;
}

// Store original content and translated content
const contentCache: { [key: string]: { original: string; translated: string } } = {};

export default function FloatingToolbar({ onClose }: FloatingToolbarProps) {
  const { user } = useAuth();
  const {
    personalizationEnabled,
    translationEnabled,
    isTranslating,
    togglePersonalization,
    toggleTranslation,
    setIsTranslating
  } = usePersonalizationSafe();

  const [translationError, setTranslationError] = useState<string>('');

  const handlePersonalizationToggle = () => {
    togglePersonalization();
  };

  const handleTranslationToggle = async () => {
    if (isTranslating) return;

    const contentArea = document.querySelector('.markdown, article, .theme-doc-markdown') as HTMLElement;
    if (!contentArea) {
      setTranslationError('Could not find content area');
      return;
    }

    const currentPath = window.location.pathname;
    const chapterId = currentPath.split('/').pop() || 'unknown';

    if (translationEnabled) {
      toggleTranslation();
      // Restore original content
      if (contentCache[chapterId]?.original) {
        contentArea.innerHTML = contentCache[chapterId].original;
        contentArea.style.direction = 'ltr';
        contentArea.style.textAlign = 'left';
      }
      return;
    }

    // Check if we have cached translation
    if (contentCache[chapterId]?.translated) {
      contentArea.innerHTML = contentCache[chapterId].translated;
      contentArea.style.direction = 'rtl';
      contentArea.style.textAlign = 'right';
      toggleTranslation();
      return;
    }

    // Translate current page content
    setIsTranslating(true);
    setTranslationError('');

    try {
      // Store original content before translation
      const originalContent = contentArea.innerHTML;
      contentCache[chapterId] = { original: originalContent, translated: '' };

      const contentToTranslate = contentArea.textContent || '';

      const response = await fetch(`${TRANSLATION_API_URL}/translate`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          chapter_id: chapterId,
          content: contentToTranslate.substring(0, 8000),
        }),
      });

      if (!response.ok) {
        const errorData = await response.json().catch(() => ({}));
        throw new Error(`Translation failed: ${errorData.detail || response.statusText}`);
      }

      const data = await response.json();

      if (data.translated_content) {
        // Cache the translated content
        contentCache[chapterId].translated = data.translated_content;

        // Apply translation to the page
        contentArea.innerHTML = data.translated_content;
        contentArea.style.direction = 'rtl';
        contentArea.style.textAlign = 'right';
        toggleTranslation();
      } else {
        throw new Error('No translated content received');
      }
    } catch (error) {
      console.error('Translation error:', error);
      setTranslationError(error instanceof Error ? error.message : 'Translation failed');
      // Restore original if translation failed
      if (contentCache[chapterId]?.original) {
        contentArea.innerHTML = contentCache[chapterId].original;
      }
    } finally {
      setIsTranslating(false);
    }
  };

  if (!user) {
    return (
      <div className={styles.toolbar}>
        <div className={styles.signInPrompt}>
          <div className={styles.promptIcon}>🔒</div>
          <div className={styles.promptText}>
            <strong>Sign in to unlock</strong>
            <span>Personalized tooltips & Urdu translation</span>
          </div>
          <a href="/signup" className={styles.signUpLink}>Get Started</a>
        </div>
      </div>
    );
  }

  return (
    <div className={styles.toolbar}>
      <div className={styles.header}>
        <span className={styles.title}>Reading Tools</span>
      </div>

      <div className={styles.controls}>
        <button
          className={`${styles.toolButton} ${personalizationEnabled ? styles.active : ''}`}
          onClick={handlePersonalizationToggle}
          title="Toggle personalized tooltips based on your experience level"
        >
          <div className={styles.buttonIcon}>
            <svg width="20" height="20" viewBox="0 0 20 20" fill="currentColor">
              <path d="M10 2a8 8 0 100 16 8 8 0 000-16zm0 14a6 6 0 110-12 6 6 0 010 12z"/>
              <circle cx="10" cy="10" r="2"/>
            </svg>
          </div>
          <div className={styles.buttonContent}>
            <span className={styles.buttonLabel}>Personalize</span>
            <span className={styles.buttonDesc}>
              {personalizationEnabled ? 'Tooltips ON' : 'Tooltips OFF'}
            </span>
          </div>
          <div className={`${styles.toggle} ${personalizationEnabled ? styles.toggleOn : ''}`}>
            <div className={styles.toggleKnob}></div>
          </div>
        </button>

        <button
          className={`${styles.toolButton} ${translationEnabled ? styles.active : ''}`}
          onClick={handleTranslationToggle}
          disabled={isTranslating}
          title="Translate content to Urdu"
        >
          <div className={styles.buttonIcon}>
            <svg width="20" height="20" viewBox="0 0 20 20" fill="currentColor">
              <path d="M10 2C5.58 2 2 5.58 2 10s3.58 8 8 8 8-3.58 8-8-3.58-8-8-8zm-1 14.93c-2.83-.48-5-2.94-5-5.93s2.17-5.45 5-5.93v11.86zm2 0V4.07c2.83.48 5 2.94 5 5.93s-2.17 5.45-5 5.93z"/>
            </svg>
          </div>
          <div className={styles.buttonContent}>
            <span className={styles.buttonLabel}>اردو (Urdu)</span>
            <span className={styles.buttonDesc}>
              {isTranslating ? 'Translating...' : translationEnabled ? 'Translation ON' : 'Translation OFF'}
            </span>
          </div>
          <div className={`${styles.toggle} ${translationEnabled ? styles.toggleOn : ''}`}>
            {isTranslating ? (
              <div className={styles.spinner}></div>
            ) : (
              <div className={styles.toggleKnob}></div>
            )}
          </div>
        </button>
      </div>

      {personalizationEnabled && (
        <div className={styles.statusCard}>
          <div className={styles.statusIcon}>✓</div>
          <div className={styles.statusText}>
            Tooltips tailored to your <strong>{user.programmingExperience}</strong> level
          </div>
        </div>
      )}

      {translationError && (
        <div className={styles.errorCard}>
          <span>⚠️ {translationError}</span>
        </div>
      )}

      <div className={styles.userInfo}>
        <div className={styles.badges}>
          <span className={styles.badge} title="Programming Experience">
            💻 {user.programmingExperience}
          </span>
          <span className={styles.badge} title="ROS Familiarity">
            🤖 {user.rosFamiliarity}
          </span>
          <span className={styles.badge} title="AI/ML Background">
            🧠 {user.aiMlBackground}
          </span>
        </div>
      </div>
    </div>
  );
}
