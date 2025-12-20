import React, { useEffect } from 'react';
import { useAuth } from '../hooks/useAuth';
import { useHistory, useLocation } from '@docusaurus/router';
import BrowserOnly from '@docusaurus/BrowserOnly';

interface ProtectedContentProps {
  children: React.ReactNode;
  chapterId: string;
}

// Freemium model: First 3 chapters + intro are free
const FREE_CHAPTERS = ['intro', 'chapter1', 'chapter2', 'chapter3'];

export default function ProtectedContent({ children, chapterId }: ProtectedContentProps) {
  return (
    <BrowserOnly fallback={<div>Loading...</div>}>
      {() => <ProtectedContentInner chapterId={chapterId}>{children}</ProtectedContentInner>}
    </BrowserOnly>
  );
}

function ProtectedContentInner({ children, chapterId }: ProtectedContentProps) {
  const { user, isLoading } = useAuth();
  const history = useHistory();
  const location = useLocation();

  const isFreemiumContent = FREE_CHAPTERS.includes(chapterId);

  // Note: We do NOT auto-redirect. Instead, we show the lock screen
  // and let users choose to sign up or sign in via the buttons.

  // Show loading state
  if (isLoading) {
    return (
      <div style={{
        textAlign: 'center',
        padding: '4rem 0',
        minHeight: '400px',
        display: 'flex',
        alignItems: 'center',
        justifyContent: 'center'
      }}>
        <div>
          <div style={{
            width: '48px',
            height: '48px',
            border: '4px solid var(--ifm-color-emphasis-300)',
            borderTopColor: '#667eea',
            borderRadius: '50%',
            animation: 'spin 1s linear infinite',
            margin: '0 auto 1rem'
          }} />
          <p style={{ color: 'var(--ifm-color-emphasis-700)' }}>Loading...</p>
        </div>
      </div>
    );
  }

  // Show premium content gate if user not logged in
  if (!user && !isFreemiumContent) {
    return (
      <div style={{
        textAlign: 'center',
        padding: '4rem 2rem',
        background: 'var(--ifm-background-surface-color)',
        borderRadius: '12px',
        margin: '2rem auto',
        maxWidth: '600px',
        border: '2px solid var(--ifm-color-emphasis-300)'
      }}>
        <div style={{ fontSize: '4rem', marginBottom: '1rem' }}>🔒</div>
        <h2 style={{
          background: 'linear-gradient(135deg, #667eea 0%, #764ba2 100%)',
          WebkitBackgroundClip: 'text',
          WebkitTextFillColor: 'transparent',
          backgroundClip: 'text',
          marginBottom: '1rem'
        }}>
          Premium Content
        </h2>
        <p style={{
          fontSize: '1.1rem',
          color: 'var(--ifm-color-emphasis-700)',
          marginBottom: '2rem'
        }}>
          This chapter requires a <strong>free account</strong> to access.
        </p>

        <div style={{
          background: 'var(--ifm-color-emphasis-100)',
          padding: '1.5rem',
          borderRadius: '8px',
          marginBottom: '2rem',
          textAlign: 'left'
        }}>
          <h3 style={{ marginTop: 0, fontSize: '1.1rem' }}>✨ Free Account Benefits:</h3>
          <ul style={{
            marginBottom: 0,
            paddingLeft: '1.5rem',
            color: 'var(--ifm-color-emphasis-800)'
          }}>
            <li>Access to all 12 chapters</li>
            <li>Personalized tooltips based on your experience level</li>
            <li>Urdu translation with AI-powered caching</li>
            <li>Progress tracking across chapters</li>
          </ul>
        </div>

        <div style={{
          display: 'flex',
          gap: '1rem',
          justifyContent: 'center',
          flexWrap: 'wrap'
        }}>
          <a
            href={`/signup?returnUrl=${encodeURIComponent(location.pathname)}`}
            style={{
              padding: '0.875rem 2rem',
              background: 'linear-gradient(135deg, #667eea 0%, #764ba2 100%)',
              color: 'white',
              borderRadius: '8px',
              textDecoration: 'none',
              fontWeight: 600,
              fontSize: '1.05rem',
              display: 'inline-block',
              transition: 'all 0.3s ease',
              boxShadow: '0 4px 12px rgba(102, 126, 234, 0.3)'
            }}
            onMouseOver={(e) => {
              e.currentTarget.style.transform = 'translateY(-2px)';
              e.currentTarget.style.boxShadow = '0 6px 20px rgba(102, 126, 234, 0.4)';
            }}
            onMouseOut={(e) => {
              e.currentTarget.style.transform = 'translateY(0)';
              e.currentTarget.style.boxShadow = '0 4px 12px rgba(102, 126, 234, 0.3)';
            }}
          >
            Sign Up Free →
          </a>

          <a
            href={`/signin?returnUrl=${encodeURIComponent(location.pathname)}`}
            style={{
              padding: '0.875rem 2rem',
              background: 'transparent',
              color: '#667eea',
              border: '2px solid #667eea',
              borderRadius: '8px',
              textDecoration: 'none',
              fontWeight: 600,
              fontSize: '1.05rem',
              display: 'inline-block',
              transition: 'all 0.3s ease'
            }}
            onMouseOver={(e) => {
              e.currentTarget.style.background = 'rgba(102, 126, 234, 0.1)';
              e.currentTarget.style.transform = 'translateY(-2px)';
            }}
            onMouseOut={(e) => {
              e.currentTarget.style.background = 'transparent';
              e.currentTarget.style.transform = 'translateY(0)';
            }}
          >
            Sign In
          </a>
        </div>

        <p style={{
          marginTop: '1.5rem',
          fontSize: '0.9rem',
          color: 'var(--ifm-color-emphasis-600)'
        }}>
          Already explored? Chapters 1-3 are <a href="/docs/intro" style={{ color: '#667eea' }}>free to access</a>
        </p>
      </div>
    );
  }

  // Render content for authenticated users or free chapters
  return <>{children}</>;
}
