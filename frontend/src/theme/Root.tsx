import React from 'react';
import ChatWidget from '../components/ChatWidget';
import { AuthProvider } from '../contexts/AuthContext';

// Default implementation, that you can customize
export default function Root({ children }) {
  return (
    <AuthProvider>
      {children}
      <ChatWidget />
    </AuthProvider>
  );
}
