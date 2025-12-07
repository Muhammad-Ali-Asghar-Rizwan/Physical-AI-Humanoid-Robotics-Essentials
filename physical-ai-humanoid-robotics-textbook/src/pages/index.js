

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
        <div className={styles.heroContent}>
          <div className={styles.heroText}>
            <h1 className="hero__title">{siteConfig.title}</h1>
            <p className="hero__subtitle">Master the future of embodied intelligence</p>
            <div className={styles.buttons}>
              <Link
                className="button button--secondary button--lg"
                to="/docs/module-1-ros2-nervous-system">
                Start Learning
              </Link>
              <Link
                className="button button--primary button--lg"
                to="/docs/module-1-ros2-nervous-system">
                View Modules
              </Link>
            </div>
          </div>
          <div className={styles.heroImage}>
            <img
              src="/img/hero-robot.jpg"
              alt="Humanoid Robot"
              className={styles.heroImagePlaceholder}
              onError={(e) => {
                e.target.onerror = null;
                e.target.src = "data:image/svg+xml,%3Csvg xmlns='http://www.w3.org/2000/svg' width='500' height='300' viewBox='0 0 500 300'%3E%3Crect width='500' height='300' fill='%234a6cf7'/%3E%3Ctext x='50%25' y='50%25' dominant-baseline='middle' text-anchor='middle' font-family='Arial' font-size='40' fill='white'%3E🤖%3C/text%3E%3C/svg%3E";
              }}
            />
          </div>
        </div>
      </div>
    </header>
  );
};

const FeatureCard = ({ title, description, icon, moduleNumber, to }) => (
  <div className={clsx('col col--3', styles.moduleCard)}>
    <div className={styles.moduleCardInner}>
      <div className={styles.moduleBadge}>{moduleNumber}</div>
      <div className={styles.moduleIcon}>{icon}</div>
      <h3>{title}</h3>
      <p>{description}</p>
      <Link className={`button button--outline button--secondary button--block ${styles.moduleButton}`} to={to}>
        Learn More
      </Link>
    </div>
  </div>
);

const LearningSection = () => (
  <section className={styles.learningSection}>
    <div className="container padding-horiz--md">
      <div className="row">
        <div className="col col--6">
          <img
            src="/img/robot-simulation.jpg"
            alt="Robot Simulation"
            className={styles.learningImagePlaceholder}
            onError={(e) => {
              e.target.onerror = null;
              e.target.src = "data:image/svg+xml,%3Csvg xmlns='http://www.w3.org/2000/svg' width='500' height='300' viewBox='0 0 500 300'%3E%3Crect width='500' height='300' fill='%236c5ce7'/%3E%3Ctext x='50%25' y='50%25' dominant-baseline='middle' text-anchor='middle' font-family='Arial' font-size='40' fill='white'%3E🎮%3C/text%3E%3C/svg%3E";
            }}
          />
        </div>
        <div className="col col--6">
          <h2>What You'll Learn</h2>
          <ul className={styles.learningList}>
            <li>Build complete humanoid robots using ROS 2</li>
            <li>Create AI controllers with NVIDIA Isaac Gym</li>
            <li>Implement vision-language-action models</li>
            <li>Design simulation-to-reality transfer systems</li>
            <li>Deploy embodied AI systems on physical hardware</li>
          </ul>
        </div>
      </div>
    </div>
  </section>
);

const TechnologiesSection = () => (
  <section className={styles.technologiesSection}>
    <div className="container padding-horiz--md">
      <h2 className={styles.sectionTitle}>Technologies Used</h2>
      <div className={styles.techGrid}>
        <div className={styles.techItem}>
          <img
            src="/img/ros-logo.png"
            alt="ROS 2"
            className={styles.techLogo}
            onError={(e) => {
              e.target.onerror = null;
              e.target.src = "data:image/svg+xml,%3Csvg xmlns='http://www.w3.org/2000/svg' width='100' height='100' viewBox='0 0 100 100'%3E%3Crect width='100' height='100' fill='%2322A5FF'/%3E%3Ctext x='50%25' y='50%25' dominant-baseline='middle' text-anchor='middle' font-family='Arial' font-size='20' fill='white'%3EROS%3C/text%3E%3C/svg%3E";
            }}
          />
          <span>ROS 2</span>
        </div>
        <div className={styles.techItem}>
          <img
            src="/img/gazebo-logo.png"
            alt="Gazebo"
            className={styles.techLogo}
            onError={(e) => {
              e.target.onerror = null;
              e.target.src = "data:image/svg+xml,%3Csvg xmlns='http://www.w3.org/2000/svg' width='100' height='100' viewBox='0 0 100 100'%3E%3Crect width='100' height='100' fill='%23FF6B35'/%3E%3Ctext x='50%25' y='50%25' dominant-baseline='middle' text-anchor='middle' font-family='Arial' font-size='20' fill='white'%3EGZ%3C/text%3E%3C/svg%3E";
            }}
          />
          <span>Gazebo</span>
        </div>
        <div className={styles.techItem}>
          <img
            src="/img/isaac-logo.png"
            alt="NVIDIA Isaac"
            className={styles.techLogo}
            onError={(e) => {
              e.target.onerror = null;
              e.target.src = "data:image/svg+xml,%3Csvg xmlns='http://www.w3.org/2000/svg' width='100' height='100' viewBox='0 0 100 100'%3E%3Crect width='100' height='100' fill='%2376B900'/%3E%3Ctext x='50%25' y='50%25' dominant-baseline='middle' text-anchor='middle' font-family='Arial' font-size='20' fill='white'%3EISAAC%3C/text%3E%3C/svg%3E";
            }}
          />
          <span>NVIDIA Isaac</span>
        </div>
        <div className={styles.techItem}>
          <img
            src="/img/openai-logo.png"
            alt="OpenAI"
            className={styles.techLogo}
            onError={(e) => {
              e.target.onerror = null;
              e.target.src = "data:image/svg+xml,%3Csvg xmlns='http://www.w3.org/2000/svg' width='100' height='100' viewBox='0 0 100 100'%3E%3Crect width='100' height='100' fill='%2310A37F'/%3E%3Ctext x='50%25' y='50%25' dominant-baseline='middle' text-anchor='middle' font-family='Arial' font-size='20' fill='white'%3EAI%3C/text%3E%3C/svg%3E";
            }}
          />
          <span>OpenAI</span>
        </div>
        <div className={styles.techItem}>
          <img
            src="/img/python-logo.png"
            alt="Python"
            className={styles.techLogo}
            onError={(e) => {
              e.target.onerror = null;
              e.target.src = "data:image/svg+xml,%3Csvg xmlns='http://www.w3.org/2000/svg' width='100' height='100' viewBox='0 0 100 100'%3E%3Crect width='100' height='100' fill='%233776AB'/%3E%3Ctext x='50%25' y='50%25' dominant-baseline='middle' text-anchor='middle' font-family='Arial' font-size='20' fill='white'%3EPY%3C/text%3E%3C/svg%3E";
            }}
          />
          <span>Python</span>
        </div>
        <div className={styles.techItem}>
          <img
            src="/img/cpp-logo.png"
            alt="C++"
            className={styles.techLogo}
            onError={(e) => {
              e.target.onerror = null;
              e.target.src = "data:image/svg+xml,%3Csvg xmlns='http://www.w3.org/2000/svg' width='100' height='100' viewBox='0 0 100 100'%3E%3Crect width='100' height='100' fill='%2300599C'/%3E%3Ctext x='50%25' y='50%25' dominant-baseline='middle' text-anchor='middle' font-family='Arial' font-size='20' fill='white'%3EC%2B%2B%3C/text%3E%3C/svg%3E";
            }}
          />
          <span>C++</span>
        </div>
        <div className={styles.techItem}>
          <img
            src="/img/tensorflow-logo.png"
            alt="TensorFlow"
            className={styles.techLogo}
            onError={(e) => {
              e.target.onerror = null;
              e.target.src = "data:image/svg+xml,%3Csvg xmlns='http://www.w3.org/2000/svg' width='100' height='100' viewBox='0 0 100 100'%3E%3Crect width='100' height='100' fill='%23FF6F00'/%3E%3Ctext x='50%25' y='50%25' dominant-baseline='middle' text-anchor='middle' font-family='Arial' font-size='20' fill='white'%3ETF%3C/text%3E%3C/svg%3E";
            }}
          />
          <span>TensorFlow</span>
        </div>
        <div className={styles.techItem}>
          <img
            src="/img/pytorch-logo.png"
            alt="PyTorch"
            className={styles.techLogo}
            onError={(e) => {
              e.target.onerror = null;
              e.target.src = "data:image/svg+xml,%3Csvg xmlns='http://www.w3.org/2000/svg' width='100' height='100' viewBox='0 0 100 100'%3E%3Crect width='100' height='100' fill='%23EE4C2C'/%3E%3Ctext x='50%25' y='50%25' dominant-baseline='middle' text-anchor='middle' font-family='Arial' font-size='20' fill='white'%3EPT%3C/text%3E%3C/svg%3E";
            }}
          />
          <span>PyTorch</span>
        </div>
      </div>
    </div>
  </section>
);

const ProjectsSection = () => (
  <section className={styles.projectsSection}>
    <div className="container padding-horiz--md">
      <h2 className={styles.sectionTitle}>Student Projects Showcase</h2>
      <div className="row">
        <div className="col col--4">
          <div className={styles.projectCard}>
            <img
              src="/img/dexterous-manipulation.jpg"
              alt="Dexterous Manipulation"
              className={styles.projectImage}
              onError={(e) => {
                e.target.onerror = null;
                e.target.src = "data:image/svg+xml,%3Csvg xmlns='http://www.w3.org/2000/svg' width='400' height='150' viewBox='0 0 400 150'%3E%3Crect width='400' height='150' fill='%234ECDC4'/%3E%3Ctext x='50%25' y='50%25' dominant-baseline='middle' text-anchor='middle' font-family='Arial' font-size='20' fill='white'%3EHand%3C/text%3E%3C/svg%3E";
              }}
            />
            <h3>Dexterous Manipulation</h3>
            <p>Robotic hand with precise object manipulation</p>
          </div>
        </div>
        <div className="col col--4">
          <div className={styles.projectCard}>
            <img
              src="/img/bipedal-locomotion.jpg"
              alt="Bipedal Locomotion"
              className={styles.projectImage}
              onError={(e) => {
                e.target.onerror = null;
                e.target.src = "data:image/svg+xml,%3Csvg xmlns='http://www.w3.org/2000/svg' width='400' height='150' viewBox='0 0 400 150'%3E%3Crect width='400' height='150' fill='%2345B7D1'/%3E%3Ctext x='50%25' y='50%25' dominant-baseline='middle' text-anchor='middle' font-family='Arial' font-size='20' fill='white'%3EWalking%3C/text%3E%3C/svg%3E";
              }}
            />
            <h3>Bipedal Locomotion</h3>
            <p>Dynamic balance control for human-like walking</p>
          </div>
        </div>
        <div className="col col--4">
          <div className={styles.projectCard}>
            <img
              src="/img/visual-navigation.jpg"
              alt="Visual Navigation"
              className={styles.projectImage}
              onError={(e) => {
                e.target.onerror = null;
                e.target.src = "data:image/svg+xml,%3Csvg xmlns='http://www.w3.org/2000/svg' width='400' height='150' viewBox='0 0 400 150'%3E%3Crect width='400' height='150' fill='%2396DEDA'/%3E%3Ctext x='50%25' y='50%25' dominant-baseline='middle' text-anchor='middle' font-family='Arial' font-size='20' fill='white'%3EVision%3C/text%3E%3C/svg%3E";
              }}
            />
            <h3>Visual Navigation</h3>
            <p>AI-powered navigation in dynamic environments</p>
          </div>
        </div>
      </div>
    </div>
  </section>
);

export default function Home() {
  const { siteConfig } = useDocusaurusContext();

  return (
    <Layout
      title={`Welcome to ${siteConfig.title}`}
      description="Physical AI & Humanoid Robotics Textbook">
      <HomepageHeader />
      <main>
        {/* Modules Grid */}
        <section className={styles.modulesSection}>
          <div className="container padding-horiz--md">
            <div className="row" style={{gap: '20px'}}>
              <FeatureCard
                title="ROS 2: Robotic Nervous System"
                description="Understanding the middleware that enables distributed robotic systems"
                icon="📡"
                moduleNumber="Module 1"
                to="/docs/module-1-ros2-nervous-system"
              />
              <FeatureCard
                title="Gazebo: Digital Twin"
                description="Creating accurate physics simulations for robot testing"
                icon="🎮"
                moduleNumber="Module 2"
                to="/docs/module-2-digital-twin"
              />
              <FeatureCard
                title="Isaac: AI Brain"
                description="Implementing AI controllers for humanoid robotics"
                icon="🧠"
                moduleNumber="Module 3"
                to="/docs/module-3-isaac-ai-brain"
              />
              <FeatureCard
                title="VLA: Vision-Language-Action"
                description="Building systems that can see, understand, and act"
                icon="👁️"
                moduleNumber="Module 4"
                to="/docs/module-4-vision-language-action"
              />
            </div>
          </div>
        </section>

        {/* Additional sections */}
        <LearningSection />
        <TechnologiesSection />
        <ProjectsSection />
      </main>
    </Layout>
  );
}

