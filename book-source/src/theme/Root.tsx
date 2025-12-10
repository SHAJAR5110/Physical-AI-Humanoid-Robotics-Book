/**
 * Root theme wrapper component for Docusaurus.
 *
 * This component wraps the entire Docusaurus application and provides
 * the ChatBot widget globally across all pages (docs, homepage, etc.).
 *
 * Docusaurus will automatically use this as the root theme provider when
 * it exists at src/theme/Root.tsx.
 */

import React from 'react';
import ChatBot from '../components/ChatBot';

interface RootProps {
  children: React.ReactNode;
}

/**
 * Root component that wraps the entire Docusaurus app.
 *
 * The ChatBot widget is rendered here so it appears on every page
 * of the documentation site.
 */
const Root: React.FC<RootProps> = ({ children }) => {
  return (
    <>
      {children}
      <ChatBot />
    </>
  );
};

export default Root;
