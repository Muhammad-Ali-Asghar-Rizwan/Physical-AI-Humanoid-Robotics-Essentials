import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import styles from './styles.module.css';

const HomepageHeader = () => {
  const { siteConfig } = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <h1 className="hero__title">{siteConfig.title}</h1>
        <p className="hero__subtitle">{siteConfig.tagline}</p>
        <div className={styles.buttons}>
          <Link
            className="button button--secondary button--lg"
            to="/docs/intro">
            Read the Textbook
          </Link>
        </div>
      </div>
    </header>
  );
};

export default function Home() {
  const { siteConfig } = useDocusaurusContext();

  return (
    <Layout
      title={`Hello from ${siteConfig.title}`}
      description="Physical AI & Humanoid Robotics Textbook">
      <HomepageHeader />
      <main>
        <section className={styles.features}>
          <div className="container">
            <div className="row">
              <div className="col col--4">
                <h2>Physical AI</h2>
                <p>Understanding the intersection of artificial intelligence with physical systems</p>
              </div>
              <div className="col col--4">
                <h2>Humanoid Robotics</h2>
                <p>Building robots that mimic human form and behavior</p>
              </div>
              <div className="col col--4">
                <h2>Ethical AI</h2>
                <p>Developing AI systems that are safe, fair, and beneficial to humanity</p>
              </div>
            </div>
          </div>
        </section>
      </main>
    </Layout>
  );
}