/**
 * Root Layout Component
 *
 * Wraps the entire Docusaurus application with global providers.
 * This is where ChatKitProvider is integrated to make the chat widget available
 * to all pages in the documentation.
 */

import React, { ReactNode } from 'react';
import { ChatKitProvider } from '@site/src/components/ChatKit/ChatKitProvider';

interface RootProps {
  children: ReactNode;
}

/**
 * Root Component
 * Top-level layout component for the entire Docusaurus site
 *
 * @param children - The page content from Docusaurus router
 */
export default function Root({ children }: RootProps): JSX.Element {
  return (
    <ChatKitProvider>
      {children}
    </ChatKitProvider>
  );
}
