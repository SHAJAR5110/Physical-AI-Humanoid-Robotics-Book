import { useState, useEffect, useCallback } from 'react';

export interface AuthUser {
  id: string;
  email: string;
  name: string;
  os?: string;
  gpu?: string;
  experience_level?: string;
  robotics_background?: boolean;
}

export interface AuthState {
  user: AuthUser | null;
  isAuthenticated: boolean;
  isLoading: boolean;
  error: string | null;
  sessionToken: string | null;
}

const API_BASE_URL = process.env.REACT_APP_API_BASE_URL || 'http://localhost:8000';
const SESSION_TOKEN_KEY = 'auth_session_token';
const USER_KEY = 'auth_user';

/**
 * useAuth Hook
 *
 * Manages user authentication state including:
 * - Sign up with profile attributes
 * - Sign in with email/password
 * - Sign out and session cleanup
 * - Persistent session storage
 * - Automatic session restoration
 */
export function useAuth() {
  const [state, setState] = useState<AuthState>({
    user: null,
    isAuthenticated: false,
    isLoading: true,
    error: null,
    sessionToken: null,
  });

  // Restore session from localStorage on mount
  useEffect(() => {
    const restoreSession = () => {
      try {
        const token = localStorage.getItem(SESSION_TOKEN_KEY);
        const userJson = localStorage.getItem(USER_KEY);

        if (token && userJson) {
          const user = JSON.parse(userJson);
          setState({
            user,
            isAuthenticated: true,
            isLoading: false,
            error: null,
            sessionToken: token,
          });
        } else {
          setState((prev) => ({
            ...prev,
            isLoading: false,
          }));
        }
      } catch (err) {
        console.error('Failed to restore session:', err);
        // Clear corrupted session
        localStorage.removeItem(SESSION_TOKEN_KEY);
        localStorage.removeItem(USER_KEY);
        setState((prev) => ({
          ...prev,
          isLoading: false,
        }));
      }
    };

    restoreSession();
  }, []);

  /**
   * Sign up with email, password, and profile
   */
  const signup = useCallback(
    async (
      email: string,
      password: string,
      name: string,
      profileData?: {
        os?: string;
        gpu?: string;
        experience_level?: string;
        robotics_background?: boolean;
      }
    ) => {
      setState((prev) => ({ ...prev, isLoading: true, error: null }));

      try {
        const response = await fetch(`${API_BASE_URL}/api/auth/signup`, {
          method: 'POST',
          headers: {
            'Content-Type': 'application/json',
          },
          body: JSON.stringify({
            email,
            password,
            name,
            ...profileData,
          }),
        });

        if (!response.ok) {
          const errorData = await response.json();
          throw new Error(errorData.error || 'Signup failed');
        }

        const data = await response.json();

        const user: AuthUser = {
          id: data.user_id,
          email: data.email,
          name: data.name,
        };

        // Store session and user data
        localStorage.setItem(SESSION_TOKEN_KEY, data.session_token);
        localStorage.setItem(USER_KEY, JSON.stringify(user));

        setState({
          user,
          isAuthenticated: true,
          isLoading: false,
          error: null,
          sessionToken: data.session_token,
        });

        return { success: true, user };
      } catch (err) {
        const errorMessage = err instanceof Error ? err.message : 'Signup failed';
        setState((prev) => ({
          ...prev,
          isLoading: false,
          error: errorMessage,
        }));
        return { success: false, error: errorMessage };
      }
    },
    []
  );

  /**
   * Sign in with email and password
   */
  const signin = useCallback(async (email: string, password: string) => {
    setState((prev) => ({ ...prev, isLoading: true, error: null }));

    try {
      const response = await fetch(`${API_BASE_URL}/api/auth/signin`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({ email, password }),
      });

      if (!response.ok) {
        const errorData = await response.json();
        throw new Error(errorData.error || 'Signin failed');
      }

      const data = await response.json();

      const user: AuthUser = {
        id: data.user_id,
        email: data.user.email,
        name: data.user.name,
        os: data.user.os,
        gpu: data.user.gpu,
        experience_level: data.user.experience_level,
        robotics_background: data.user.robotics_background,
      };

      // Store session and user data
      localStorage.setItem(SESSION_TOKEN_KEY, data.session_token);
      localStorage.setItem(USER_KEY, JSON.stringify(user));

      setState({
        user,
        isAuthenticated: true,
        isLoading: false,
        error: null,
        sessionToken: data.session_token,
      });

      return { success: true, user };
    } catch (err) {
      const errorMessage = err instanceof Error ? err.message : 'Signin failed';
      setState((prev) => ({
        ...prev,
        isLoading: false,
        error: errorMessage,
      }));
      return { success: false, error: errorMessage };
    }
  }, []);

  /**
   * Sign out and clear session
   */
  const signout = useCallback(async () => {
    setState((prev) => ({ ...prev, isLoading: true, error: null }));

    try {
      const token = state.sessionToken;

      if (token) {
        await fetch(`${API_BASE_URL}/api/auth/signout`, {
          method: 'POST',
          headers: {
            'Content-Type': 'application/json',
            'Authorization': `Bearer ${token}`,
          },
        });
      }

      // Clear localStorage
      localStorage.removeItem(SESSION_TOKEN_KEY);
      localStorage.removeItem(USER_KEY);

      setState({
        user: null,
        isAuthenticated: false,
        isLoading: false,
        error: null,
        sessionToken: null,
      });

      return { success: true };
    } catch (err) {
      const errorMessage = err instanceof Error ? err.message : 'Signout failed';
      setState((prev) => ({
        ...prev,
        isLoading: false,
        error: errorMessage,
      }));
      return { success: false, error: errorMessage };
    }
  }, [state.sessionToken]);

  /**
   * Update user profile
   */
  const updateProfile = useCallback(
    async (updates: Partial<AuthUser>) => {
      setState((prev) => ({ ...prev, isLoading: true, error: null }));

      try {
        const token = state.sessionToken;

        if (!token) {
          throw new Error('Not authenticated');
        }

        const response = await fetch(`${API_BASE_URL}/api/auth/profile`, {
          method: 'PATCH',
          headers: {
            'Content-Type': 'application/json',
            'Authorization': `Bearer ${token}`,
          },
          body: JSON.stringify(updates),
        });

        if (!response.ok) {
          const errorData = await response.json();
          throw new Error(errorData.error || 'Profile update failed');
        }

        const updatedUser = await response.json();

        // Update localStorage
        localStorage.setItem(USER_KEY, JSON.stringify(updatedUser));

        setState((prev) => ({
          ...prev,
          user: updatedUser,
          isLoading: false,
          error: null,
        }));

        return { success: true, user: updatedUser };
      } catch (err) {
        const errorMessage = err instanceof Error ? err.message : 'Profile update failed';
        setState((prev) => ({
          ...prev,
          isLoading: false,
          error: errorMessage,
        }));
        return { success: false, error: errorMessage };
      }
    },
    [state.sessionToken]
  );

  return {
    ...state,
    signup,
    signin,
    signout,
    updateProfile,
  };
}
