import React, { useState } from 'react';
import { useAuth } from '@site/src/hooks/useAuth';
import styles from './AuthForm.module.css';

interface SignInFormProps {
  onSuccess?: () => void;
  onSignUpClick?: () => void;
}

export default function SignInForm({ onSuccess, onSignUpClick }: SignInFormProps) {
  const { signin, isLoading, error } = useAuth();

  const [formData, setFormData] = useState({
    email: '',
    password: '',
  });

  const [formError, setFormError] = useState<string | null>(null);

  const handleChange = (e: React.ChangeEvent<HTMLInputElement>) => {
    const { name, value } = e.target;
    setFormData((prev) => ({
      ...prev,
      [name]: value,
    }));
  };

  const validateForm = () => {
    if (!formData.email || !formData.password) {
      setFormError('Email and password are required');
      return false;
    }

    setFormError(null);
    return true;
  };

  const handleSubmit = async (e: React.FormEvent<HTMLFormElement>) => {
    e.preventDefault();

    if (!validateForm()) {
      return;
    }

    const result = await signin(formData.email, formData.password);

    if (result.success) {
      onSuccess?.();
    }
  };

  return (
    <div className={styles.formContainer}>
      <h2>Sign In</h2>
      <p className={styles.subtitle}>Welcome back to Physical AI Book</p>

      {(formError || error) && (
        <div className={styles.errorAlert}>
          {formError || error}
        </div>
      )}

      <form onSubmit={handleSubmit} className={styles.form}>
        {/* Email Field */}
        <div className={styles.formGroup}>
          <label htmlFor="email">Email *</label>
          <input
            type="email"
            id="email"
            name="email"
            value={formData.email}
            onChange={handleChange}
            placeholder="you@example.com"
            required
          />
        </div>

        {/* Password Field */}
        <div className={styles.formGroup}>
          <label htmlFor="password">Password *</label>
          <input
            type="password"
            id="password"
            name="password"
            value={formData.password}
            onChange={handleChange}
            placeholder="••••••••"
            required
          />
        </div>

        {/* Remember Me */}
        <div className={styles.formGroup}>
          <label className={styles.checkboxLabel}>
            <input type="checkbox" defaultChecked />
            <span>Remember me</span>
          </label>
        </div>

        {/* Submit Button */}
        <button
          type="submit"
          className={styles.submitButton}
          disabled={isLoading}
        >
          {isLoading ? 'Signing In...' : 'Sign In'}
        </button>
      </form>

      {/* Sign Up Link */}
      <p className={styles.switchAuth}>
        Don't have an account?{' '}
        <button
          type="button"
          className={styles.linkButton}
          onClick={onSignUpClick}
        >
          Sign Up
        </button>
      </p>
    </div>
  );
}
