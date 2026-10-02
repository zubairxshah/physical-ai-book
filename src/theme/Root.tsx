import React from 'react';
import { PersonalizationProvider } from '../context/PersonalizationContext';

// The chatbot widget is loaded via the `scripts` entry in docusaurus.config.ts.
export default function Root({children}) {
  return (
    <PersonalizationProvider>
      {children}
    </PersonalizationProvider>
  );
}
