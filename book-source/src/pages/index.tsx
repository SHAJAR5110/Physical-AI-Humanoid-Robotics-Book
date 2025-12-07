import type {ReactNode} from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import HomepageFeatures from '@site/src/components/HomepageFeatures';
import Heading from '@theme/Heading';

import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={styles.heroBanner}>
      <div className={styles.heroContent}>
        <div className={styles.heroText}>
          <Heading as="h1" className={styles.heroTitle}>
            {siteConfig.title}
          </Heading>
          <p className={styles.heroSubtitle}>{siteConfig.tagline}</p>
          <p className={styles.heroDescription}>
            Master Physical AI and humanoid robotics from fundamentals to deployment.
            This comprehensive guide covers ROS 2, Gazebo simulation, NVIDIA Isaac platform,
            vision-language-action models, and real-world robotics systems.
            Whether you're a student, researcher, or engineer, learn the complete
            stack to build intelligent robots that perceive, reason, and act in the physical world.
          </p>
          <div className={styles.buttons}>
            <Link
              className={clsx('button', styles.primaryButton)}
              to="/docs/intro">
              📚 Start Reading the Book
            </Link>
            <Link
              className={clsx('button', styles.secondaryButton)}
              to="/docs/intro">
              🚀 Explore All Chapters
            </Link>
          </div>
        </div>
        <div className={styles.heroArt}>
          <div className={styles.gradientOrb}>
            <svg viewBox="0 0 200 200" className={styles.orbSvg}>
              <defs>
                <linearGradient id="orbGradient" x1="0%" y1="0%" x2="100%" y2="100%">
                  <stop offset="0%" style={{stopColor: '#0ea5e9', stopOpacity: 0.8}} />
                  <stop offset="100%" style={{stopColor: '#8b5cf6', stopOpacity: 0.8}} />
                </linearGradient>
              </defs>
              <circle cx="100" cy="100" r="80" fill="url(#orbGradient)" />
              <circle cx="100" cy="100" r="70" fill="none" stroke="#0ea5e9" strokeWidth="2" opacity="0.3" />
              <circle cx="100" cy="100" r="60" fill="none" stroke="#8b5cf6" strokeWidth="2" opacity="0.2" />
              <text x="100" y="110" textAnchor="middle" fontSize="48" fill="white" fontWeight="bold">🤖</text>
            </svg>
          </div>
        </div>
      </div>
    </header>
  );
}

function BookOverview() {
  return (
    <section className={styles.overview}>
      <div className="container">
        <div className={styles.overviewContent}>
          <div className={styles.overviewItem}>
            <div className={styles.overviewIcon}>📖</div>
            <h3>6 Comprehensive Chapters</h3>
            <p>17,500+ words of in-depth content covering theory, implementation, and best practices</p>
          </div>
          <div className={styles.overviewItem}>
            <div className={styles.overviewIcon}>💻</div>
            <h3>45+ Code Examples</h3>
            <p>Python, YAML, and JSON examples you can run immediately with explanations</p>
          </div>
          <div className={styles.overviewItem}>
            <div className={styles.overviewIcon}>🎯</div>
            <h3>Production-Ready</h3>
            <p>From simulation to real hardware - learn the complete robotics stack</p>
          </div>
          <div className={styles.overviewItem}>
            <div className={styles.overviewIcon}>🚀</div>
            <h3>Hands-On Projects</h3>
            <p>Build real robotic systems with ROS 2, Gazebo, and AI integration</p>
          </div>
          <div className={styles.overviewItem}>
            <div className={styles.overviewIcon}>🎓</div>
            <h3>Open Source</h3>
            <p>MIT licensed - free for educational, research, and commercial use</p>
          </div>
          <div className={styles.overviewItem}>
            <div className={styles.overviewIcon}>🤝</div>
            <h3>Community Driven</h3>
            <p>Contribute, suggest improvements, and help others learn robotics</p>
          </div>
        </div>
      </div>
    </section>
  );
}

export default function Home(): ReactNode {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={siteConfig.title}
      description="Master Physical AI and humanoid robotics with our comprehensive guide">
      <HomepageHeader />
      <main>
        <BookOverview />
        <HomepageFeatures />
      </main>
    </Layout>
  );
}
