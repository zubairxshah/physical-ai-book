import { useEffect } from 'react';
import { create } from 'zustand';
import { persist } from 'zustand/middleware';
import { AUTH_API_URL } from '../config/api';

export interface User {
  id: string;
  email: string;
  name: string;
  programmingExperience: 'beginner' | 'intermediate' | 'advanced';
  rosFamiliarity: 'none' | 'basic' | 'intermediate' | 'expert';
  aiMlBackground: 'none' | 'basic' | 'intermediate' | 'expert';
  enablePersonalization: boolean;
  enableUrduTranslation: boolean;
}

interface AuthState {
  user: User | null;
  isLoading: boolean;
  setUser: (user: User | null) => void;
  signOut: () => Promise<void>;
}

export const useAuthStore = create<AuthState>()(
  persist(
    (set) => ({
      user: null,
      isLoading: true,

      setUser: (user) => set({ user, isLoading: false }),

      signOut: async () => {
        try {
          await fetch(`${AUTH_API_URL}/sign-out`, {
            method: 'POST',
            credentials: 'include'
          });
          set({ user: null });
        } catch (error) {
          console.error('Sign out error:', error);
          // Still clear local state even if API call fails
          set({ user: null });
        }
      }
    }),
    {
      name: 'physical-ai-auth-storage',
      partialize: (state) => ({ user: state.user }) // Only persist user, not isLoading
    }
  )
);

export function useAuth() {
  const { user, isLoading, setUser, signOut } = useAuthStore();

  useEffect(() => {
    async function checkSession() {
      try {
        const res = await fetch(`${AUTH_API_URL}/session`, {
          credentials: 'include'
        });

        if (res.ok) {
          const data = await res.json();
          if (data.user) {
            setUser(data.user);
          } else {
            setUser(null);
          }
        } else {
          setUser(null);
        }
      } catch (error) {
        console.error('Session check failed:', error);
        setUser(null);
      }
    }

    checkSession();
  }, [setUser]);

  return { user, isLoading, signOut };
}
