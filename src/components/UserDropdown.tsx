import React, { useState, useRef, useEffect } from 'react';
import { useAuth } from '../hooks/useAuth';
import { useHistory } from '@docusaurus/router';
import FloatingToolbar from './FloatingToolbar';
import styles from './UserDropdown.module.css';

export default function UserDropdown() {
  const { user, signOut } = useAuth();
  const [isOpen, setIsOpen] = useState(false);
  const [showToolbar, setShowToolbar] = useState(false);
  const dropdownRef = useRef<HTMLDivElement>(null);
  const toolbarRef = useRef<HTMLDivElement>(null);
  const toolbarTimeoutRef = useRef<NodeJS.Timeout | null>(null);
  const history = useHistory();

  useEffect(() => {
    function handleClickOutside(event: MouseEvent) {
      if (dropdownRef.current && !dropdownRef.current.contains(event.target as Node)) {
        setIsOpen(false);
      }
      if (toolbarRef.current && !toolbarRef.current.contains(event.target as Node)) {
        setShowToolbar(false);
      }
    }

    if (isOpen || showToolbar) {
      document.addEventListener('mousedown', handleClickOutside);
    }

    return () => {
      document.removeEventListener('mousedown', handleClickOutside);
    };
  }, [isOpen, showToolbar]);

  // Cleanup timeout on unmount
  useEffect(() => {
    return () => {
      if (toolbarTimeoutRef.current) {
        clearTimeout(toolbarTimeoutRef.current);
      }
    };
  }, []);

  const handleToolbarMouseEnter = () => {
    if (toolbarTimeoutRef.current) {
      clearTimeout(toolbarTimeoutRef.current);
      toolbarTimeoutRef.current = null;
    }
    setShowToolbar(true);
  };

  const handleToolbarMouseLeave = () => {
    toolbarTimeoutRef.current = setTimeout(() => {
      setShowToolbar(false);
    }, 300); // Small delay to allow moving to the dropdown
  };

  if (!user) {
    return (
      <div className={styles.navActions}>
        {/* Tools button for non-logged in users - shows sign up prompt */}
        <div
          className={styles.toolsWrapper}
          ref={toolbarRef}
          onMouseEnter={handleToolbarMouseEnter}
          onMouseLeave={handleToolbarMouseLeave}
        >
          <button
            className={styles.toolsButton}
            aria-label="Reading tools"
            aria-expanded={showToolbar}
          >
            <svg width="20" height="20" viewBox="0 0 20 20" fill="currentColor">
              <path d="M11.49 3.17c-.38-1.56-2.6-1.56-2.98 0a1.532 1.532 0 01-2.286.948c-1.372-.836-2.942.734-2.106 2.106.54.886.061 2.042-.947 2.287-1.561.379-1.561 2.6 0 2.978a1.532 1.532 0 01.947 2.287c-.836 1.372.734 2.942 2.106 2.106a1.532 1.532 0 012.287.947c.379 1.561 2.6 1.561 2.978 0a1.533 1.533 0 012.287-.947c1.372.836 2.942-.734 2.106-2.106a1.533 1.533 0 01.947-2.287c1.561-.379 1.561-2.6 0-2.978a1.532 1.532 0 01-.947-2.287c.836-1.372-.734-2.942-2.106-2.106a1.532 1.532 0 01-2.287-.947zM10 13a3 3 0 100-6 3 3 0 000 6z"/>
            </svg>
            <span>Tools</span>
          </button>
          {showToolbar && (
            <div className={styles.toolbarDropdown}>
              <FloatingToolbar />
            </div>
          )}
        </div>
        <div className={styles.authButtons}>
          <a href="/signin" className={styles.signInButton}>Sign In</a>
          <a href="/signup" className={styles.signUpButton}>Sign Up</a>
        </div>
      </div>
    );
  }

  const handleSignOut = async () => {
    await signOut();
    setIsOpen(false);
    history.push('/');
  };

  return (
    <div className={styles.navActions}>
      {/* Tools button with hover dropdown */}
      <div
        className={styles.toolsWrapper}
        ref={toolbarRef}
        onMouseEnter={handleToolbarMouseEnter}
        onMouseLeave={handleToolbarMouseLeave}
      >
        <button
          className={styles.toolsButton}
          aria-label="Reading tools"
          aria-expanded={showToolbar}
        >
          <svg width="20" height="20" viewBox="0 0 20 20" fill="currentColor">
            <path d="M11.49 3.17c-.38-1.56-2.6-1.56-2.98 0a1.532 1.532 0 01-2.286.948c-1.372-.836-2.942.734-2.106 2.106.54.886.061 2.042-.947 2.287-1.561.379-1.561 2.6 0 2.978a1.532 1.532 0 01.947 2.287c-.836 1.372.734 2.942 2.106 2.106a1.532 1.532 0 012.287.947c.379 1.561 2.6 1.561 2.978 0a1.533 1.533 0 012.287-.947c1.372.836 2.942-.734 2.106-2.106a1.533 1.533 0 01.947-2.287c1.561-.379 1.561-2.6 0-2.978a1.532 1.532 0 01-.947-2.287c.836-1.372-.734-2.942-2.106-2.106a1.532 1.532 0 01-2.287-.947zM10 13a3 3 0 100-6 3 3 0 000 6z"/>
          </svg>
          <span>Tools</span>
        </button>
        {showToolbar && (
          <div className={styles.toolbarDropdown}>
            <FloatingToolbar />
          </div>
        )}
      </div>

      {/* User dropdown */}
      <div className={styles.userDropdown} ref={dropdownRef}>
        <button
          className={styles.userButton}
          onClick={() => setIsOpen(!isOpen)}
          aria-label="User menu"
          aria-expanded={isOpen}
        >
          <div className={styles.userAvatar}>
            {user.name.charAt(0).toUpperCase()}
          </div>
          <span className={styles.userName}>{user.name}</span>
          <svg
            width="16"
            height="16"
            viewBox="0 0 16 16"
            fill="currentColor"
            className={isOpen ? styles.chevronUp : styles.chevronDown}
          >
            <path d="M4 6l4 4 4-4H4z"/>
          </svg>
        </button>

        {isOpen && (
          <div className={styles.dropdown}>
            <div className={styles.dropdownHeader}>
              <div className={styles.userInfo}>
                <div className={styles.userName}>{user.name}</div>
                <div className={styles.userEmail}>{user.email}</div>
              </div>
            </div>

            <div className={styles.dropdownDivider} />

            <div className={styles.dropdownSection}>
              <div className={styles.sectionTitle}>Experience Level</div>
              <div className={styles.badges}>
                <span className={styles.badge}>
                  💻 Programming: {user.programmingExperience}
                </span>
                <span className={styles.badge}>
                  🤖 ROS: {user.rosFamiliarity}
                </span>
                <span className={styles.badge}>
                  🧠 AI/ML: {user.aiMlBackground}
                </span>
              </div>
            </div>

            <div className={styles.dropdownDivider} />

            <button
              className={styles.dropdownItem}
              onClick={handleSignOut}
            >
              <svg width="16" height="16" viewBox="0 0 16 16" fill="currentColor">
                <path d="M3 0a2 2 0 00-2 2v12a2 2 0 002 2h8a2 2 0 002-2V2a2 2 0 00-2-2H3zm5 10.5l-3.5-3L8 4v2.5h5v2H8V10.5z"/>
              </svg>
              Sign Out
            </button>
          </div>
        )}
      </div>
    </div>
  );
}
