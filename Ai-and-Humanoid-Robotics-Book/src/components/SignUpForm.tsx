import React, { useState } from 'react';
import { useAuth } from '@site/src/hooks/useAuth';
import styles from './AuthForm.module.css';

interface SignUpFormProps {
  onSuccess?: () => void;
  onSignInClick?: () => void;
}

export default function SignUpForm({ onSuccess, onSignInClick }: SignUpFormProps) {
  const { signup, isLoading, error } = useAuth();

  const [formData, setFormData] = useState({
    email: '',
    password: '',
    confirmPassword: '',
    name: '',
    os: 'linux',
    gpu: '',
    experience_level: 'beginner',
    robotics_background: false,
  });

  const [formError, setFormError] = useState<string | null>(null);

  const handleChange = (
    e: React.ChangeEvent<HTMLInputElement | HTMLSelectElement | HTMLTextAreaElement>
  ) => {
    const { name, value, type } = e.target;

    if (type === 'checkbox') {
      const checkbox = e.target as HTMLInputElement;
      setFormData((prev) => ({
        ...prev,
        [name]: checkbox.checked,
      }));
    } else {
      setFormData((prev) => ({
        ...prev,
        [name]: value,
      }));
    }
  };

  const validateForm = () => {
    if (!formData.email || !formData.password || !formData.name) {
      setFormError('Email, password, and name are required');
      return false;
    }

    if (formData.password.length < 8) {
      setFormError('Password must be at least 8 characters');
      return false;
    }

    if (formData.password !== formData.confirmPassword) {
      setFormError('Passwords do not match');
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

    const result = await signup(
      formData.email,
      formData.password,
      formData.name,
      {
        os: formData.os,
        gpu: formData.gpu || undefined,
        experience_level: formData.experience_level,
        robotics_background: formData.robotics_background,
      }
    );

    if (result.success) {
      onSuccess?.();
    }
  };

  return (
    <div className={styles.formContainer}>
      <h2>Create Account</h2>
      <p className={styles.subtitle}>Join the Physical AI community</p>

      {(formError || error) && (
        <div className={styles.errorAlert}>
          {formError || error}
        </div>
      )}

      <form onSubmit={handleSubmit} className={styles.form}>
        {/* Name Field */}
        <div className={styles.formGroup}>
          <label htmlFor="name">Full Name *</label>
          <input
            type="text"
            id="name"
            name="name"
            value={formData.name}
            onChange={handleChange}
            placeholder="John Doe"
            required
          />
        </div>

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
          <label htmlFor="password">Password (min 8 characters) *</label>
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

        {/* Confirm Password Field */}
        <div className={styles.formGroup}>
          <label htmlFor="confirmPassword">Confirm Password *</label>
          <input
            type="password"
            id="confirmPassword"
            name="confirmPassword"
            value={formData.confirmPassword}
            onChange={handleChange}
            placeholder="••••••••"
            required
          />
        </div>

        {/* Operating System */}
        <div className={styles.formGroup}>
          <label htmlFor="os">Operating System</label>
          <select
            id="os"
            name="os"
            value={formData.os}
            onChange={handleChange}
          >
            <option value="linux">Linux</option>
            <option value="macos">macOS</option>
            <option value="windows">Windows</option>
          </select>
        </div>

        {/* GPU */}
        <div className={styles.formGroup}>
          <label htmlFor="gpu">GPU (Optional)</label>
          <input
            type="text"
            id="gpu"
            name="gpu"
            value={formData.gpu}
            onChange={handleChange}
            placeholder="e.g., NVIDIA RTX 4090, Apple M3"
          />
        </div>

        {/* Experience Level */}
        <div className={styles.formGroup}>
          <label htmlFor="experience_level">Experience Level</label>
          <select
            id="experience_level"
            name="experience_level"
            value={formData.experience_level}
            onChange={handleChange}
          >
            <option value="beginner">Beginner</option>
            <option value="intermediate">Intermediate</option>
            <option value="advanced">Advanced</option>
          </select>
        </div>

        {/* Robotics Background */}
        <div className={styles.formGroup}>
          <label htmlFor="robotics_background" className={styles.checkboxLabel}>
            <input
              type="checkbox"
              id="robotics_background"
              name="robotics_background"
              checked={formData.robotics_background}
              onChange={handleChange}
            />
            <span>I have robotics experience</span>
          </label>
        </div>

        {/* Submit Button */}
        <button
          type="submit"
          className={styles.submitButton}
          disabled={isLoading}
        >
          {isLoading ? 'Creating Account...' : 'Create Account'}
        </button>
      </form>

      {/* Sign In Link */}
      <p className={styles.switchAuth}>
        Already have an account?{' '}
        <button
          type="button"
          className={styles.linkButton}
          onClick={onSignInClick}
        >
          Sign In
        </button>
      </p>
    </div>
  );
}
