import React, { createContext, useContext, useState, useCallback, ReactNode } from 'react';

interface PersonalizationContextType {
  personalizationEnabled: boolean;
  translationEnabled: boolean;
  isTranslating: boolean;
  togglePersonalization: () => void;
  toggleTranslation: () => void;
  setIsTranslating: (value: boolean) => void;
}

const PersonalizationContext = createContext<PersonalizationContextType | undefined>(undefined);

export function PersonalizationProvider({ children }: { children: ReactNode }) {
  const [personalizationEnabled, setPersonalizationEnabled] = useState(false);
  const [translationEnabled, setTranslationEnabled] = useState(false);
  const [isTranslating, setIsTranslating] = useState(false);

  const togglePersonalization = useCallback(() => {
    setPersonalizationEnabled(prev => !prev);
  }, []);

  const toggleTranslation = useCallback(() => {
    setTranslationEnabled(prev => !prev);
  }, []);

  return (
    <PersonalizationContext.Provider
      value={{
        personalizationEnabled,
        translationEnabled,
        isTranslating,
        togglePersonalization,
        toggleTranslation,
        setIsTranslating,
      }}
    >
      {children}
    </PersonalizationContext.Provider>
  );
}

export function usePersonalization() {
  const context = useContext(PersonalizationContext);
  if (context === undefined) {
    throw new Error('usePersonalization must be used within a PersonalizationProvider');
  }
  return context;
}

// Safe hook that returns defaults if outside provider (for SSR compatibility)
export function usePersonalizationSafe() {
  const context = useContext(PersonalizationContext);
  if (context === undefined) {
    return {
      personalizationEnabled: false,
      translationEnabled: false,
      isTranslating: false,
      togglePersonalization: () => {},
      toggleTranslation: () => {},
      setIsTranslating: () => {},
    };
  }
  return context;
}
