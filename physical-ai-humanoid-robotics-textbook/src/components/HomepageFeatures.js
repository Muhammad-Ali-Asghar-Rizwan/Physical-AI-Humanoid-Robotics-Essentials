import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useBaseUrl from '@docusaurus/useBaseUrl';
import styles from './HomepageFeatures.module.css';

const FeatureList = [
  {
    title: 'Physical AI Fundamentals',
    Svg: require('../../static/img/robot.jpg').default,
    description: (
      <>
        Explore the foundations of Physical AI, where artificial intelligence meets the physical world. 
        Learn how robots perceive, reason, and act in real environments.
      </>
    ),
    link: '/docs/module-1-ros2-nervous-system',
    icon: '🧠'
  },
  {
    title: 'ROS 2 Robotics Framework',
    Svg: require('../../static/img/ros-logo.svg').default,
    description: (
      <>
        Master the Robot Operating System (ROS 2), the standard framework for robotics development. 
        Control robots with nodes, topics, and services.
      </>
    ),
    link: '/docs/module-1-ros2-nervous-system',
    icon: '🤖'
  },
  {
    title: 'NVIDIA Isaac Platform',
    Svg: require('../../static/img/isaac-logo.svg').default,
    description: (
      <>
        Leverage NVIDIA Isaac for GPU-accelerated robotics. Build advanced perception systems 
        with Isaac Sim and Isaac ROS GEMs.
      </>
    ),
    link: '/docs/module-3-isaac-ai-brain',
    icon: '🎮'
  },
  {
    title: 'Humanoid Robot Design',
    Svg: require('../../static/img/humanoid-robot.svg').default,
    description: (
      <>
        Understand the mechanics and control of humanoid robots. Learn about bipedal locomotion, 
        balance, and human-like movement patterns.
      </>
    ),
    link: '/docs/module-2-digital-twin',
    icon: '🚶'
  },
  {
    title: 'Vision-Language-Action',
    Svg: require('../../static/img/vla-logo.svg').default,
    description: (
      <>
        Integrate vision, language, and action systems. Create robots that understand natural 
        language commands and execute them physically.
      </>
    ),
    link: '/docs/module-4-vision-language-action',
    icon: '🗣️'
  },
  {
    title: 'Simulation to Reality',
    Svg: require('../../static/img/sim2real.svg').default,
    description: (
      <>
        Bridge the gap between simulation and real-world deployment. Learn techniques for 
        transferring learned behaviors to physical robots.
      </>
    ),
    link: '/docs/module-2-digital-twin',
    icon: '🔄'
  },
];

function Feature({ Svg, title, description, link, icon }) {
  return (
    <Link to={useBaseUrl(link)} className="col col--4 margin-bottom--lg">
      <div className={clsx('feature-card', styles.featureCard)}>
        <div className="feature-icon">
          <span style={{ fontSize: '1.8rem' }}>{icon}</span>
        </div>
        <h3>{title}</h3>
        <p>{description}</p>
      </div>
    </Link>
  );
}

export default function HomepageFeatures() {
  return (
    <section className={styles.features}>
      <div className="container">
        <div className="row">
          {FeatureList.map((props, idx) => (
            <Feature key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}