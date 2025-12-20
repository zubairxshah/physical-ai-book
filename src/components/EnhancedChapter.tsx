import React, { useState, useEffect } from 'react';
import { useAuth } from '../hooks/useAuth';
import { TRANSLATION_API_URL } from '../config/api';
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
      // Extract text content from the chapter
      const chapterElement = document.querySelector(`[data-chapter-id="${chapterId}"]`);
      const contentToTranslate = chapterElement?.textContent || '';

      console.log('Translation request:', {
        chapterId,
        contentLength: contentToTranslate.length,
        contentPreview: contentToTranslate.substring(0, 100)
      });

      const response = await fetch(`${TRANSLATION_API_URL}/translate`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          chapter_id: chapterId,
          content: contentToTranslate,
        }),
      });

      if (!response.ok) {
        const errorData = await response.json().catch(() => ({}));
        console.error('Translation API error:', errorData);
        throw new Error(`Translation failed: ${errorData.detail || response.statusText}`);
      }

      const data = await response.json();
      setTranslatedContent(data.translated_content);
      setTranslationEnabled(true);
    } catch (error) {
      console.error('Translation error:', error);
      setTranslationError(error instanceof Error ? error.message : 'Translation failed');
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
