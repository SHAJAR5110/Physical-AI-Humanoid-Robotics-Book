import React, { useState } from 'react';
import Layout from '@theme/Layout';
import Link from '@docusaurus/Link';
import SignUpForm from '@site/src/components/SignUpForm';
import SignInForm from '@site/src/components/SignInForm';
import styles from './auth.module.css';

type AuthMode = 'signin' | 'signup';

export default function AuthPage(): React.ReactNode {
  const [authMode, setAuthMode] = useState<AuthMode>('signin');

  return (
    <Layout
      title="Sign In or Create Account"
      description="Authentication for Physical AI & Humanoid Robotics Book"
    >
      <div className={styles.authPageContainer}>
        <div className={styles.authHeader}>
          <Link to="/" className={styles.logo}>
            ← Back to Home
          </Link>
          <h1>Physical AI Book</h1>
          <p>Sign in to access personalized content and track your progress</p>
        </div>

        <div className={styles.authContent}>
          <div className={styles.tabButtons}>
            <button
              className={`${styles.tabButton} ${
                authMode === 'signin' ? styles.active : ''
              }`}
              onClick={() => setAuthMode('signin')}
            >
              Sign In
            </button>
            <button
              className={`${styles.tabButton} ${
                authMode === 'signup' ? styles.active : ''
              }`}
              onClick={() => setAuthMode('signup')}
            >
              Create Account
            </button>
          </div>

          <div className={styles.formWrapper}>
            {authMode === 'signin' ? (
              <SignInForm
                onSignUpClick={() => setAuthMode('signup')}
                onSuccess={() => {
                  // Redirect to book after successful signin
                  window.location.href = '/docs/intro';
                }}
              />
            ) : (
              <SignUpForm
                onSignInClick={() => setAuthMode('signin')}
                onSuccess={() => {
                  // Redirect to book after successful signup
                  window.location.href = '/docs/intro';
                }}
              />
            )}
          </div>

          <div className={styles.authInfo}>
            <h3>Why create an account?</h3>
            <ul>
              <li>📚 <strong>Personalized Learning</strong> - Content adapted to your skill level and background</li>
              <li>🌍 <strong>Multiple Languages</strong> - Read chapters in Urdu or other languages</li>
              <li>💬 <strong>AI Chatbot</strong> - Ask questions about book content with proper citations</li>
              <li>📝 <strong>Progress Tracking</strong> - Keep track of which chapters you've read</li>
              <li>🔧 <strong>Customization</strong> - Save your OS, GPU, and experience level preferences</li>
            </ul>
          </div>
        </div>
      </div>
    </Layout>
  );
}
