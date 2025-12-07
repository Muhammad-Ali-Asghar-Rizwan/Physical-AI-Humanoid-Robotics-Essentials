import React, { useState, useEffect } from 'react';
import { useLocation } from '@docusaurus/router';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import styles from './ReadingProgressBar.module.css';

const ReadingProgressBar = () => {
  const [readingProgress, setReadingProgress] = useState(0);
  const location = useLocation();
  const { siteConfig } = useDocusaurusContext();

  useEffect(() => {
    const updateReadingProgress = () => {
      const scrollPosition = window.scrollY;
      const windowHeight = window.innerHeight;
      const documentHeight = document.documentElement.scrollHeight;
      const maxScroll = documentHeight - windowHeight;
      
      if (maxScroll > 0) {
        const progress = Math.round((scrollPosition / maxScroll) * 100);
        setReadingProgress(progress);
      } else {
        setReadingProgress(0);
      }
    };

    // Only show progress bar on doc pages
    const isDocPage = location.pathname.includes('/docs/');
    
    if (isDocPage) {
      window.addEventListener('scroll', updateReadingProgress);
      updateReadingProgress(); // Initial calculation
      
      return () => {
        window.removeEventListener('scroll', updateReadingProgress);
      };
    }
  }, [location]);

  // Only show progress bar on doc pages
  const isDocPage = location.pathname.includes('/docs/');
  
  if (!isDocPage) {
    return null;
  }

  return (
    <div className={styles.readingProgressBarContainer}>
      <div 
        className={styles.readingProgressBar} 
        style={{ width: `${readingProgress}%` }}
      />
    </div>
  );
};

export default ReadingProgressBar;