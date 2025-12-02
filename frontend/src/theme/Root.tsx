import React from 'react';
import ChatWidget from '../components/ChatWidget';
import { AuthProvider } from '../contexts/AuthContext';
import { PersonalizationProvider } from '../contexts/PersonalizationContext';
import PersonalizationGuard from '../components/Auth/PersonalizationGuard';

// Default implementation, that you can customize
export default function Root({ children }) {
  return (
    <AuthProvider>
      <PersonalizationProvider>
        <PersonalizationGuard>
          {children}
        </PersonalizationGuard>
        <ChatWidget />
      </PersonalizationProvider>
    </AuthProvider>
  );
}
