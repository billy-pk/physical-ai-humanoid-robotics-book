import type {ReactNode} from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import HomepageFeatures from '@site/src/components/HomepageFeatures';
import Heading from '@theme/Heading';
import {usePersonalization} from '../contexts/PersonalizationContext';

import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <Heading as="h1" className="hero__title">
          {siteConfig.title}
        </Heading>
        <p className="hero__subtitle">{siteConfig.tagline}</p>
        <div className={styles.buttons}>
          <Link
            className="button button--secondary button--lg"
            to="/docs/intro">
            Start Learning →
          </Link>
        </div>
      </div>
    </header>
  );
}

export default function Home(): ReactNode {
  const {siteConfig} = useDocusaurusContext();
  const {preferences, isLoading} = usePersonalization();
  
  return (
    <Layout
      title={`Hello from ${siteConfig.title}`}
      description="Description will go into a meta tag in <head />">
      <HomepageHeader />
      <main>
        {/* Preferences Status Section */}
        <section style={{ padding: '2rem 0', backgroundColor: 'var(--ifm-background-surface-color)' }}>
          <div className="container">
            <div style={{ 
              padding: '1.5rem', 
              borderRadius: '8px', 
              border: '1px solid var(--ifm-color-emphasis-300)',
              backgroundColor: 'var(--ifm-background-color)'
            }}>
              <h2 style={{ marginBottom: '1rem' }}>Your Learning Preferences</h2>
              {isLoading ? (
                <p>Loading preferences...</p>
              ) : preferences ? (
                <div style={{ display: 'flex', flexDirection: 'column', gap: '0.5rem' }}>
                  <p><strong>Experience Level:</strong> {preferences.experience_level}</p>
                  <p><strong>Content Mode:</strong> {preferences.content_mode === 'personalized' ? 'Personalized' : 'Full Content'}</p>
                  <p><strong>Learning Topics:</strong> {preferences.learning_topics.join(', ')}</p>
                  <p><strong>Urdu Translation:</strong> {preferences.urdu_translation_enabled ? 'Enabled' : 'Disabled'}</p>
                  <Link
                    className="button button--primary button--sm"
                    to="/settings"
                    style={{ marginTop: '1rem', width: 'fit-content' }}>
                    Manage Preferences →
                  </Link>
                </div>
              ) : (
                <div>
                  <p>You haven't set your learning preferences yet. Customize your experience to get personalized content!</p>
                  <Link
                    className="button button--primary button--sm"
                    to="/popup"
                    style={{ marginTop: '1rem', width: 'fit-content' }}>
                    Set Preferences →
                  </Link>
                </div>
              )}
            </div>
          </div>
        </section>
        <HomepageFeatures />
      </main>
    </Layout>
  );
}
