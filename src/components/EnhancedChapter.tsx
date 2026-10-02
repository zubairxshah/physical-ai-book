import React, { useState, useEffect } from 'react';
import { useAuth } from '../hooks/useAuth';
import { translateElementToUrdu } from '../utils/translateToUrdu';
import ChapterControls from './ChapterControls';

interface EnhancedChapterProps {
  chapterId: string;
  originalContent: React.ReactNode;
  children: React.ReactNode;
}

export default function EnhancedChapter({
  chapterId,
  originalContent,
  children
}: EnhancedChapterProps) {
  const { user } = useAuth();
  const [personalizationEnabled, setPersonalizationEnabled] = useState(false);
  const [translationEnabled, setTranslationEnabled] = useState(false);
  const [isTranslating, setIsTranslating] = useState(false);
  const [translatedContent, setTranslatedContent] = useState<string>('');
  const [translationError, setTranslationError] = useState<string>('');

  // Handle personalization toggle
  const handlePersonalizationToggle = (enabled: boolean) => {
    setPersonalizationEnabled(enabled);
    // Personalization is handled by PersonalizedTooltip components
    // which automatically use the user's experience level
  };

  // Handle translation toggle
  const handleTranslationToggle = async (enabled: boolean) => {
    if (!enabled) {
      setTranslationEnabled(false);
      setTranslatedContent('');
      setTranslationError('');
      return;
    }

    // If we already have translation cached, just show it
    if (translatedContent) {
      setTranslationEnabled(true);
      return;
    }

    // Otherwise, fetch translation
    setIsTranslating(true);
    setTranslationError('');

    try {
      const chapterElement = document.querySelector(`[data-chapter-id="${chapterId}"]`) as HTMLElement | null;
      if (!chapterElement) {
        throw new Error('Could not find chapter content');
      }

      // Safe to render as HTML: structure is the original chapter, translations are inserted as text
      setTranslatedContent(await translateElementToUrdu(chapterElement, chapterId));
      setTranslationEnabled(true);
    } catch (error) {
      console.error('Translation error:', error);
      setTranslationError(`Translation failed: ${error instanceof Error ? error.message : 'unknown error'}`);
    } finally {
      setIsTranslating(false);
    }
  };

  return (
    <div>
      <ChapterControls
        onPersonalizationToggle={handlePersonalizationToggle}
        onTranslationToggle={handleTranslationToggle}
        isTranslating={isTranslating}
      />

      {translationError && (
        <div style={{
          background: '#fff3cd',
          border: '1px solid #ffc107',
          borderRadius: '8px',
          padding: '12px 16px',
          marginBottom: '20px',
          color: '#856404'
        }}>
          <strong>Translation Error:</strong> {translationError}
        </div>
      )}

      <div
        data-chapter-id={chapterId}
        style={{
          direction: translationEnabled && translatedContent ? 'rtl' : 'ltr',
          textAlign: translationEnabled && translatedContent ? 'right' : 'left',
        }}
      >
        {translationEnabled && translatedContent ? (
          <div
            dangerouslySetInnerHTML={{ __html: translatedContent }}
            style={{ fontFamily: 'Arial, sans-serif' }}
          />
        ) : (
          children
        )}
      </div>
    </div>
  );
}
