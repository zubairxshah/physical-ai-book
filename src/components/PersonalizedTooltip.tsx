import React, { useState, useEffect } from 'react';
import { useAuth } from '../hooks/useAuth';
import { usePersonalizationSafe } from '../context/PersonalizationContext';
import { PERSONALIZATION_API_URL } from '../config/api';
import styles from './PersonalizedTooltip.module.css';

interface PersonalizedTooltipProps {
  term: string;
  children: React.ReactNode;
  chapterId?: string;
}

export default function PersonalizedTooltip({
  term,
  children,
  chapterId
}: PersonalizedTooltipProps) {
  const { user } = useAuth();
  const { personalizationEnabled } = usePersonalizationSafe();
  const [definition, setDefinition] = useState<string | null>(null);
  const [isLoading, setIsLoading] = useState(false);
  const [showTooltip, setShowTooltip] = useState(false);
  const [error, setError] = useState(false);

  // Get chapter ID from URL if not provided
  const effectiveChapterId = chapterId || (typeof window !== 'undefined'
    ? window.location.pathname.split('/').pop() || 'unknown'
    : 'unknown');

  useEffect(() => {
    // Reset when term changes or tooltip is hidden
    if (!showTooltip) {
      setDefinition(null);
      setError(false);
    }
  }, [showTooltip, term]);

  useEffect(() => {
    // Fetch definition when tooltip is shown and personalization is enabled
    if (showTooltip && !definition && !error && personalizationEnabled && user) {
      fetchDefinition();
    }
  }, [showTooltip, definition, error, personalizationEnabled, user]);

  const fetchDefinition = async () => {
    if (!user) return;

    setIsLoading(true);
    setError(false);

    try {
      const response = await fetch(`${PERSONALIZATION_API_URL}/tooltips`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        credentials: 'include',
        body: JSON.stringify({
          term,
          chapter_id: effectiveChapterId,
          user_profile: {
            programming_experience: user.programmingExperience,
            ros_familiarity: user.rosFamiliarity,
            ai_ml_background: user.aiMlBackground
          }
        })
      });

      if (response.ok) {
        const data = await response.json();
        setDefinition(data.definition);
      } else {
        // Tooltip not found in database
        setError(true);
        console.warn(`Tooltip not found for term: ${term}`);
      }
    } catch (err) {
      console.error('Failed to fetch tooltip:', err);
      setError(true);
    } finally {
      setIsLoading(false);
    }
  };

  // If user not logged in or personalization disabled via global toolbar, render children without tooltip styling
  if (!user || !personalizationEnabled) {
    return <>{children}</>;
  }

  return (
    <span
      className={styles.tooltipTrigger}
      onMouseEnter={() => setShowTooltip(true)}
      onMouseLeave={() => setShowTooltip(false)}
      onFocus={() => setShowTooltip(true)}
      onBlur={() => setShowTooltip(false)}
      tabIndex={0}
      role="button"
      aria-label={`Show definition for ${term}`}
    >
      {children}

      {showTooltip && (
        <span className={styles.tooltipBox} role="tooltip">
          {isLoading ? (
            <span className={styles.loading}>
              <span className={styles.loadingSpinner}></span>
              Loading...
            </span>
          ) : definition ? (
            <span>{definition}</span>
          ) : error ? (
            <span style={{ fontStyle: 'italic', opacity: 0.8 }}>
              Definition not available
            </span>
          ) : null}
        </span>
      )}
    </span>
  );
}
