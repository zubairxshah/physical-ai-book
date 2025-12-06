import React, { useEffect } from 'react';

export default function Root({children}) {
  useEffect(() => {
    // Wait for DOM to be fully ready
    const loadChatbot = () => {
      const script = document.createElement('script');
      script.src = '/physical-ai-book/chatbot-widget-v2.js';
      script.async = false; // Load synchronously to ensure DOM is ready
      script.onload = () => {
        console.log('✅ Chatbot widget loaded successfully');
        console.log('Click the 💬 button to open chat');
      };
      script.onerror = (error) => {
        console.error('❌ Failed to load chatbot widget:', error);
      };
      document.body.appendChild(script);
    };

    // Ensure DOM is fully loaded
    if (document.readyState === 'complete') {
      loadChatbot();
    } else {
      window.addEventListener('load', loadChatbot);
      return () => window.removeEventListener('load', loadChatbot);
    }
  }, []);

  return <>{children}</>;
}
