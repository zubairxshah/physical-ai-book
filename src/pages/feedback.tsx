import React, { useState, useEffect } from 'react';
import Layout from '@theme/Layout';
import BrowserOnly from '@docusaurus/BrowserOnly';
import styles from './feedback.module.css';

const API_URL = typeof window !== 'undefined' && window.location.hostname === 'localhost'
  ? 'http://localhost:8003'
  : 'https://engisoft-physical-ai-backend.hf.space';

function FeedbackForm() {
  const [user, setUser] = useState<any>(null);
  const [loading, setLoading] = useState(true);
  const [feedbackType, setFeedbackType] = useState('suggestion');
  const [message, setMessage] = useState('');
  const [rating, setRating] = useState(5);
  const [submitting, setSubmitting] = useState(false);
  const [submitted, setSubmitted] = useState(false);
  const [error, setError] = useState('');

  useEffect(() => {
    // Check if user is logged in
    const checkSession = async () => {
      try {
        const response = await fetch(`${API_URL}/session`, {
          credentials: 'include',
        });
        const data = await response.json();
        setUser(data.user);
      } catch (err) {
        console.error('Session check failed:', err);
      } finally {
        setLoading(false);
      }
    };
    checkSession();
  }, []);

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    if (!user) return;

    setSubmitting(true);
    setError('');

    try {
      const response = await fetch(`${API_URL}/feedback`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        credentials: 'include',
        body: JSON.stringify({
          user_id: user.id,
          user_email: user.email,
          feedback_type: feedbackType,
          message,
          rating,
        }),
      });

      if (!response.ok) {
        const data = await response.json();
        throw new Error(data.detail || 'Failed to submit feedback');
      }

      setSubmitted(true);
      setMessage('');
      setRating(5);
    } catch (err: any) {
      setError(err.message || 'Failed to submit feedback');
    } finally {
      setSubmitting(false);
    }
  };

  if (loading) {
    return (
      <div className={styles.container}>
        <div className={styles.loading}>Loading...</div>
      </div>
    );
  }

  if (!user) {
    return (
      <div className={styles.container}>
        <div className={styles.card}>
          <div className={styles.lockIcon}>🔒</div>
          <h2>Sign In Required</h2>
          <p>Please sign in to submit feedback. We value your input!</p>
          <a href="/signin?returnUrl=/feedback" className={styles.signInButton}>
            Sign In to Continue
          </a>
          <p className={styles.signUpPrompt}>
            Don't have an account? <a href="/signup">Sign up for free</a>
          </p>
        </div>
      </div>
    );
  }

  if (submitted) {
    return (
      <div className={styles.container}>
        <div className={styles.card}>
          <div className={styles.successIcon}>✅</div>
          <h2>Thank You!</h2>
          <p>Your feedback has been submitted successfully. We appreciate your input!</p>
          <button
            onClick={() => setSubmitted(false)}
            className={styles.submitButton}
          >
            Submit More Feedback
          </button>
        </div>
      </div>
    );
  }

  return (
    <div className={styles.container}>
      <div className={styles.card}>
        <div className={styles.header}>
          <h1>Share Your Feedback</h1>
          <p>Help us improve the Physical AI Book experience</p>
        </div>

        <form onSubmit={handleSubmit} className={styles.form}>
          <div className={styles.userInfo}>
            <span>Submitting as: <strong>{user.email}</strong></span>
          </div>

          <div className={styles.formGroup}>
            <label>Feedback Type</label>
            <select
              value={feedbackType}
              onChange={(e) => setFeedbackType(e.target.value)}
              className={styles.select}
            >
              <option value="suggestion">💡 Suggestion</option>
              <option value="bug">🐛 Bug Report</option>
              <option value="content">📚 Content Feedback</option>
              <option value="feature">✨ Feature Request</option>
              <option value="other">📝 Other</option>
            </select>
          </div>

          <div className={styles.formGroup}>
            <label>Rating</label>
            <div className={styles.ratingContainer}>
              {[1, 2, 3, 4, 5].map((star) => (
                <button
                  key={star}
                  type="button"
                  onClick={() => setRating(star)}
                  className={`${styles.starButton} ${star <= rating ? styles.starActive : ''}`}
                >
                  ⭐
                </button>
              ))}
              <span className={styles.ratingText}>{rating}/5</span>
            </div>
          </div>

          <div className={styles.formGroup}>
            <label>Your Feedback</label>
            <textarea
              value={message}
              onChange={(e) => setMessage(e.target.value)}
              placeholder="Tell us what you think... What did you like? What could be improved?"
              required
              minLength={10}
              maxLength={2000}
              rows={6}
              className={styles.textarea}
            />
            <span className={styles.charCount}>{message.length}/2000</span>
          </div>

          {error && <div className={styles.error}>{error}</div>}

          <button
            type="submit"
            disabled={submitting || message.length < 10}
            className={styles.submitButton}
          >
            {submitting ? 'Submitting...' : 'Submit Feedback'}
          </button>
        </form>
      </div>
    </div>
  );
}

export default function FeedbackPage() {
  return (
    <Layout title="Feedback" description="Share your feedback about the Physical AI Book">
      <BrowserOnly fallback={<div style={{minHeight: '60vh', display: 'flex', alignItems: 'center', justifyContent: 'center'}}>Loading...</div>}>
        {() => <FeedbackForm />}
      </BrowserOnly>
    </Layout>
  );
}
