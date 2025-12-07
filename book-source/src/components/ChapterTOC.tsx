import React, { useState } from 'react';
import styles from './ChapterTOC.module.css';

interface HeadingItem {
  id: string;
  level: number;
  text: string;
  children?: HeadingItem[];
}

interface Chapter {
  id: string;
  number: number;
  title: string;
  headings: HeadingItem[];
}

const chapters: Chapter[] = [
  {
    id: 'intro',
    number: 1,
    title: 'Introduction to Physical AI',
    headings: [
      { id: 'what-is-physical-ai', level: 2, text: 'What is Physical AI?' },
      { id: 'why-physical-ai-matters', level: 3, text: 'Why Physical AI Matters' },
      { id: 'historical-context', level: 2, text: 'Historical Context' },
      { id: 'early-robotics', level: 3, text: 'Early Robotics (1960s-1990s)' },
      { id: 'modern-robotics', level: 3, text: 'Modern Robotics (2000s-2010s)' },
      { id: 'physical-ai-era', level: 3, text: 'Physical AI Era (2020s+)' },
      { id: 'key-technologies', level: 2, text: 'Key Technologies in Physical AI' },
      { id: 'hardware', level: 3, text: 'Hardware' },
      { id: 'software-algorithms', level: 3, text: 'Software & Algorithms' },
      { id: 'integration-frameworks', level: 3, text: 'Integration Frameworks' },
      { id: 'course-overview', level: 2, text: 'Course Overview' },
      { id: 'learning-outcomes', level: 2, text: 'Learning Outcomes' },
      { id: 'prerequisites', level: 2, text: 'Prerequisites' },
      { id: 'how-to-use', level: 2, text: 'How to Use This Book' },
    ],
  },
  {
    id: 'ros2',
    number: 2,
    title: 'ROS 2 Fundamentals',
    headings: [
      { id: 'ros2-intro', level: 2, text: 'Introduction to ROS 2' },
      { id: 'ros2-architecture', level: 2, text: 'ROS 2 Architecture' },
      { id: 'nodes-topics', level: 2, text: 'Nodes and Topics' },
      { id: 'services-actions', level: 2, text: 'Services and Actions' },
      { id: 'ros2-tools', level: 2, text: 'Essential ROS 2 Tools' },
      { id: 'ros2-examples', level: 2, text: 'Practical Examples' },
    ],
  },
  {
    id: 'gazebo',
    number: 3,
    title: 'Gazebo Simulation',
    headings: [
      { id: 'gazebo-intro', level: 2, text: 'Introduction to Gazebo' },
      { id: 'world-modeling', level: 2, text: 'World Modeling' },
      { id: 'physics-simulation', level: 2, text: 'Physics Simulation' },
      { id: 'sensors-simulation', level: 2, text: 'Sensors and Plugins' },
      { id: 'gazebo-examples', level: 2, text: 'Practical Examples' },
    ],
  },
  {
    id: 'isaac',
    number: 4,
    title: 'NVIDIA Isaac Platform',
    headings: [
      { id: 'isaac-intro', level: 2, text: 'Introduction to NVIDIA Isaac' },
      { id: 'isaac-sim', level: 2, text: 'Isaac Sim Environment' },
      { id: 'digital-twins', level: 2, text: 'Digital Twins' },
      { id: 'isaac-gem', level: 2, text: 'Isaac GEM Libraries' },
      { id: 'isaac-examples', level: 2, text: 'Practical Examples' },
    ],
  },
  {
    id: 'vla',
    number: 5,
    title: 'Vision-Language-Action Models',
    headings: [
      { id: 'vla-intro', level: 2, text: 'Introduction to VLAs' },
      { id: 'multimodal-learning', level: 2, text: 'Multimodal Learning' },
      { id: 'vla-architectures', level: 2, text: 'VLA Architectures' },
      { id: 'training-vlms', level: 2, text: 'Training Vision-Language Models' },
      { id: 'deploying-vlms', level: 2, text: 'Deploying on Hardware' },
      { id: 'vla-examples', level: 2, text: 'Practical Examples' },
    ],
  },
  {
    id: 'capstone',
    number: 6,
    title: 'Capstone Project',
    headings: [
      { id: 'capstone-overview', level: 2, text: 'Project Overview' },
      { id: 'project-architecture', level: 2, text: 'System Architecture' },
      { id: 'hardware-setup', level: 2, text: 'Hardware Setup' },
      { id: 'software-stack', level: 2, text: 'Software Stack' },
      { id: 'integration-testing', level: 2, text: 'Integration and Testing' },
      { id: 'deployment', level: 2, text: 'Deployment to Hardware' },
      { id: 'future-improvements', level: 2, text: 'Future Improvements' },
    ],
  },
];

export default function ChapterTOC(): JSX.Element {
  const [expandedChapters, setExpandedChapters] = useState<Set<string>>(new Set(['intro']));

  const toggleChapter = (chapterId: string) => {
    const newExpanded = new Set(expandedChapters);
    if (newExpanded.has(chapterId)) {
      newExpanded.delete(chapterId);
    } else {
      newExpanded.add(chapterId);
    }
    setExpandedChapters(newExpanded);
  };

  const scrollToHeading = (headingId: string) => {
    const element = document.getElementById(headingId);
    if (element) {
      element.scrollIntoView({ behavior: 'smooth' });
    }
  };

  return (
    <div className={styles.tocContainer}>
      <div className={styles.tocHeader}>
        <h3>📖 Chapters</h3>
        <p className={styles.tocSubtitle}>Quick Navigation</p>
      </div>

      <div className={styles.chaptersList}>
        {chapters.map((chapter) => (
          <div key={chapter.id} className={styles.chapterItem}>
            <button
              className={styles.chapterButton}
              onClick={() => toggleChapter(chapter.id)}
              aria-expanded={expandedChapters.has(chapter.id)}
            >
              <span className={styles.chapterNumber}>Ch {chapter.number}</span>
              <span className={styles.chapterTitle}>{chapter.title}</span>
              <span className={styles.toggleIcon}>
                {expandedChapters.has(chapter.id) ? '▼' : '▶'}
              </span>
            </button>

            {expandedChapters.has(chapter.id) && (
              <div className={styles.headingsList}>
                {chapter.headings.map((heading) => (
                  <button
                    key={heading.id}
                    className={`${styles.headingItem} ${styles[`level${heading.level}`]}`}
                    onClick={() => scrollToHeading(heading.id)}
                  >
                    {heading.text}
                  </button>
                ))}
              </div>
            )}
          </div>
        ))}
      </div>
    </div>
  );
}
