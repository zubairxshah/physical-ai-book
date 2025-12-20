import React, { useState, useEffect } from 'react';
import { useHistory, useLocation } from '@docusaurus/router';
import { AUTH_API_URL } from '../../config/api';
import styles from './AuthForms.module.css';

export default function SigninForm() {
  const history = useHistory();
  const location = useLocation();
  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');
  const [error, setError] = useState('');
  const [message, setMessage] = useState('');
  const [isLoading, setIsLoading] = useState(false);

  useEffect(() => {
    // Check for success message in URL params
    const params = new URLSearchParams(location.search);
    const urlMessage = params.get('message');
    if (urlMessage) {
      setMessage(urlMessage);
    }
  }, [location]);

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setIsLoading(true);
    setError('');
    setMessage('');

    try {
      const response = await fetch(`${AUTH_API_URL}/sign-in`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        credentials: 'include',
        body: JSON.stringify({ email, password })
      });

      const data = await response.json();

      if (!response.ok) {
        throw new Error(data.error || 'Sign in failed');
      }

      // Redirect to first protected chapter or return URL
      const params = new URLSearchParams(location.search);
      const returnUrl = params.get('returnUrl');
      history.push(returnUrl || '/docs/chapter4');

    } catch (err: any) {
      setError(err.message || 'An error occurred during sign in');
    } finally {
      setIsLoading(false);
    }
  };

  return (
    <div className={styles.authContainer}>
      <div className={styles.authCard}>
        <div className={styles.authHeader}>
          <h1>Welcome Back</h1>
          <p>Sign in to continue your learning journey</p>
        </div>

        <form onSubmit={handleSubmit} className={styles.authForm}>
          {message && (
            <div style={{
              padding: '1rem',
              background: 'rgba(102, 126, 234, 0.1)',
              borderLeft: '4px solid #667eea',
              borderRadius: '4px',
              color: 'var(--ifm-font-color-base)',
              fontSize: '0.95rem',
              margin: '-0.5rem 0 0 0'
            }}>
              {message}
            </div>
          )}

          <div className={styles.formGroup}>
            <label>Email</label>
            <input
              type="email"
              value={email}
              onChange={(e) => setEmail(e.target.value)}
              required
              placeholder="john@example.com"
              autoComplete="email"
            />
          </div>

          <div className={styles.formGroup}>
            <label>Password</label>
            <input
              type="password"
              value={password}
              onChange={(e) => setPassword(e.target.value)}
              required
              placeholder="Enter your password"
              autoComplete="current-password"
            />
          </div>

          {error && <div className={styles.error}>{error}</div>}

          <button
            type="submit"
            className={styles.submitButton}
            disabled={isLoading}
          >
            {isLoading ? 'Signing In...' : 'Sign In →'}
          </button>

          <div className={styles.authLinks}>
            <a href="/reset-password">Forgot password?</a>
            <span>·</span>
            <a href="/signup">Create account</a>
          </div>
        </form>
      </div>
    </div>
  );
}
