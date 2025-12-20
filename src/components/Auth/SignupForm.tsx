import React, { useState } from 'react';
import { useHistory } from '@docusaurus/router';
import { AUTH_API_URL } from '../../config/api';
import styles from './AuthForms.module.css';

interface SignupData {
  name: string;
  email: string;
  password: string;
  confirmPassword: string;
}

interface BackgroundData {
  programmingExperience: 'beginner' | 'intermediate' | 'advanced';
  rosFamiliarity: 'none' | 'basic' | 'intermediate' | 'expert';
  aiMlBackground: 'none' | 'basic' | 'intermediate' | 'expert';
}

export default function SignupForm() {
  const history = useHistory();
  const [step, setStep] = useState(1);
  const [signupData, setSignupData] = useState<SignupData>({
    name: '',
    email: '',
    password: '',
    confirmPassword: ''
  });
  const [backgroundData, setBackgroundData] = useState<BackgroundData>({
    programmingExperience: 'beginner',
    rosFamiliarity: 'none',
    aiMlBackground: 'none'
  });
  const [error, setError] = useState('');
  const [isLoading, setIsLoading] = useState(false);

  const handleStep1Submit = (e: React.FormEvent) => {
    e.preventDefault();

    if (signupData.password !== signupData.confirmPassword) {
      setError('Passwords do not match');
      return;
    }

    if (signupData.password.length < 8) {
      setError('Password must be at least 8 characters');
      return;
    }

    setError('');
    setStep(2);
  };

  const handleStep2Submit = async (e: React.FormEvent) => {
    e.preventDefault();
    setIsLoading(true);
    setError('');

    try {
      const response = await fetch(`${AUTH_API_URL}/sign-up`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        credentials: 'include',
        body: JSON.stringify({
          name: signupData.name,
          email: signupData.email,
          password: signupData.password,
          ...backgroundData
        })
      });

      const data = await response.json();

      if (!response.ok) {
        throw new Error(data.error || 'Signup failed');
      }

      // Auto sign-in after signup
      const signinResponse = await fetch(`${AUTH_API_URL}/sign-in`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        credentials: 'include',
        body: JSON.stringify({
          email: signupData.email,
          password: signupData.password
        })
      });

      if (signinResponse.ok) {
        // Redirect to first protected chapter
        history.push('/docs/chapter4');
      } else {
        // Signup succeeded but signin failed - redirect to signin page
        history.push('/signin?message=Please sign in with your new account');
      }

    } catch (err: any) {
      setError(err.message || 'An error occurred during signup');
    } finally {
      setIsLoading(false);
    }
  };

  return (
    <div className={styles.authContainer}>
      <div className={styles.authCard}>
        <div className={styles.authHeader}>
          <h1>Create Your Account</h1>
          <p>Step {step} of 2</p>
        </div>

        {step === 1 && (
          <form onSubmit={handleStep1Submit} className={styles.authForm}>
            <div className={styles.formGroup}>
              <label>Full Name</label>
              <input
                type="text"
                value={signupData.name}
                onChange={(e) => setSignupData({ ...signupData, name: e.target.value })}
                required
                placeholder="John Doe"
                autoComplete="name"
              />
            </div>

            <div className={styles.formGroup}>
              <label>Email</label>
              <input
                type="email"
                value={signupData.email}
                onChange={(e) => setSignupData({ ...signupData, email: e.target.value })}
                required
                placeholder="john@example.com"
                autoComplete="email"
              />
            </div>

            <div className={styles.formGroup}>
              <label>Password</label>
              <input
                type="password"
                value={signupData.password}
                onChange={(e) => setSignupData({ ...signupData, password: e.target.value })}
                required
                minLength={8}
                placeholder="At least 8 characters"
                autoComplete="new-password"
              />
            </div>

            <div className={styles.formGroup}>
              <label>Confirm Password</label>
              <input
                type="password"
                value={signupData.confirmPassword}
                onChange={(e) => setSignupData({ ...signupData, confirmPassword: e.target.value })}
                required
                placeholder="Re-enter password"
                autoComplete="new-password"
              />
            </div>

            {error && <div className={styles.error}>{error}</div>}

            <button type="submit" className={styles.submitButton}>
              Next: Background Questions →
            </button>

            <p className={styles.authLink}>
              Already have an account? <a href="/signin">Sign In</a>
            </p>
          </form>
        )}

        {step === 2 && (
          <form onSubmit={handleStep2Submit} className={styles.authForm}>
            <p className={styles.questionnaireIntro}>
              Help us personalize your learning experience by telling us about your background:
            </p>

            <div className={styles.formGroup}>
              <label>Programming Experience</label>
              <select
                value={backgroundData.programmingExperience}
                onChange={(e) => setBackgroundData({
                  ...backgroundData,
                  programmingExperience: e.target.value as any
                })}
              >
                <option value="beginner">Beginner (Just starting)</option>
                <option value="intermediate">Intermediate (1-3 years)</option>
                <option value="advanced">Advanced (3+ years)</option>
              </select>
            </div>

            <div className={styles.formGroup}>
              <label>ROS (Robot Operating System) Familiarity</label>
              <select
                value={backgroundData.rosFamiliarity}
                onChange={(e) => setBackgroundData({
                  ...backgroundData,
                  rosFamiliarity: e.target.value as any
                })}
              >
                <option value="none">None (Never used it)</option>
                <option value="basic">Basic (Heard of it, some tutorials)</option>
                <option value="intermediate">Intermediate (Built projects)</option>
                <option value="expert">Expert (Professional experience)</option>
              </select>
            </div>

            <div className={styles.formGroup}>
              <label>AI/Machine Learning Background</label>
              <select
                value={backgroundData.aiMlBackground}
                onChange={(e) => setBackgroundData({
                  ...backgroundData,
                  aiMlBackground: e.target.value as any
                })}
              >
                <option value="none">None (Curious to learn)</option>
                <option value="basic">Basic (Online courses, tutorials)</option>
                <option value="intermediate">Intermediate (Built ML models)</option>
                <option value="expert">Expert (Professional AI/ML work)</option>
              </select>
            </div>

            {error && <div className={styles.error}>{error}</div>}

            <div className={styles.formActions}>
              <button
                type="button"
                onClick={() => setStep(1)}
                className={styles.backButton}
                disabled={isLoading}
              >
                ← Back
              </button>

              <button
                type="submit"
                className={styles.submitButton}
                disabled={isLoading}
              >
                {isLoading ? 'Creating Account...' : 'Complete Signup →'}
              </button>
            </div>
          </form>
        )}
      </div>
    </div>
  );
}
