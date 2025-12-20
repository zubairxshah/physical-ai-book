import React from 'react';
import Layout from '@theme/Layout';
import Link from '@docusaurus/Link';
import styles from './index.module.css';

function HomepageHero() {
  return (
    <div className={styles.heroContainer}>
      {/* Background decorative elements */}
      <div className={styles.heroDecoration}>
        <div className={styles.heroCircle1}></div>
        <div className={styles.heroCircle2}></div>
        <div className={styles.heroCircle3}></div>
      </div>

      <div className={styles.heroText}>
        <h1 className={styles.heroTitle}>
          Physical AI & Humanoid Robotics
        </h1>
        <h2 className={styles.heroSubtitle}>
          Master the future of robotics through comprehensive guides, working examples, and AI-powered assistance.
        </h2>

        {/* Tech badges */}
        <div className={styles.techBadges}>
          <span className={styles.badge}>ROS 2</span>
          <span className={styles.badge}>Isaac Sim</span>
          <span className={styles.badge}>VLA Models</span>
          <span className={styles.badge}>Gazebo</span>
        </div>
      </div>
    </div>
  );
}

function NavigationCard({ title, links, className = '' }) {
  return (
    <div className={`${styles.card} ${className}`}>
      <h3 className={styles.cardTitle}>{title}</h3>
      <div className={styles.cardLinks}>
        {links.map((link, idx) => (
          <Link key={idx} to={link.href} className={styles.cardLink}>
            {link.label}
            <svg width="16" height="16" viewBox="0 0 16 16" fill="none" className={styles.arrow}>
              <path d="M4 12L12 4M12 4H5.6M12 4V10.4" stroke="currentColor" strokeWidth="1.5" strokeLinecap="round" strokeLinejoin="round"/>
            </svg>
          </Link>
        ))}
      </div>
    </div>
  );
}

function CTACard() {
  return (
    <div className={styles.ctaCard}>
      <h3 className={styles.ctaTitle}>Start Your Physical AI Journey</h3>
      <Link className={styles.ctaButton} to="/docs/intro">
        Get Started
      </Link>
    </div>
  );
}

function FeatureCard({ icon, text }) {
  return (
    <div className={styles.featureCard}>
      {icon}
      <span>{text}</span>
    </div>
  );
}

function RobotSVG() {
  return (
    <svg viewBox="0 0 200 200" fill="none" xmlns="http://www.w3.org/2000/svg" className={styles.robotSvg}>
      {/* Head */}
      <rect x="70" y="30" width="60" height="50" rx="8" fill="white" opacity="0.9"/>
      <circle cx="85" cy="50" r="5" fill="#667eea"/>
      <circle cx="115" cy="50" r="5" fill="#667eea"/>
      <rect x="90" y="65" width="20" height="3" rx="1.5" fill="#667eea"/>

      {/* Antenna */}
      <line x1="100" y1="30" x2="100" y2="15" stroke="white" strokeWidth="2"/>
      <circle cx="100" cy="12" r="4" fill="#f093fb"/>

      {/* Body */}
      <rect x="60" y="85" width="80" height="70" rx="10" fill="white" opacity="0.9"/>
      <circle cx="100" cy="120" r="8" fill="#764ba2"/>
      <rect x="75" y="105" width="10" height="3" rx="1.5" fill="#667eea"/>
      <rect x="115" y="105" width="10" height="3" rx="1.5" fill="#667eea"/>

      {/* Arms */}
      <rect x="35" y="90" width="20" height="50" rx="8" fill="white" opacity="0.9"/>
      <rect x="145" y="90" width="20" height="50" rx="8" fill="white" opacity="0.9"/>
      <circle cx="45" cy="145" r="6" fill="#f093fb"/>
      <circle cx="155" cy="145" r="6" fill="#f093fb"/>

      {/* Legs */}
      <rect x="75" y="160" width="18" height="30" rx="6" fill="white" opacity="0.9"/>
      <rect x="107" y="160" width="18" height="30" rx="6" fill="white" opacity="0.9"/>

      {/* Sparkles */}
      <circle cx="30" cy="60" r="2" fill="#f093fb" opacity="0.6"/>
      <circle cx="170" cy="100" r="2" fill="#f093fb" opacity="0.6"/>
      <circle cx="50" cy="170" r="2" fill="#667eea" opacity="0.6"/>
      <circle cx="150" cy="180" r="2" fill="#667eea" opacity="0.6"/>
    </svg>
  );
}

function WhySection() {
  return (
    <div className={styles.whySection}>
      <div className={styles.sectionHeader}>
        <h2 className={styles.sectionTitle}>Why Physical AI?</h2>
        <h3 className={styles.sectionSubtitle}>
          The convergence of AI, robotics, and real-world interaction
        </h3>
      </div>

      <div className={styles.whyContent}>
        <div className={styles.whyImage}>
          <div className={styles.imageCard}>
            <RobotSVG />
          </div>
        </div>

        <div className={styles.featureCards}>
          <FeatureCard
            icon={
              <svg width="32" height="32" viewBox="0 0 32 32" fill="none">
                <path d="M17.3337 3.99902V13.3324H25.3337L14.667 27.999V18.6657H6.66699L17.3337 3.99902Z"
                      stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"/>
              </svg>
            }
            text="Real-time perception and decision making"
          />
          <FeatureCard
            icon={
              <svg width="32" height="32" viewBox="0 0 32 32" fill="none">
                <path d="M12.5664 12H15.5996M12.5664 17.333H22.5171M12.5664 22.667H22.5171"
                      stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"/>
                <rect x="8.76855" y="3.67871" width="20.6312" height="24.6722" rx="2"
                      stroke="currentColor" strokeWidth="2"/>
              </svg>
            }
            text="Complete code examples with ROS 2 & Isaac Sim"
          />
          <FeatureCard
            icon={
              <svg width="32" height="32" viewBox="0 0 32 32" fill="none">
                <path d="M16 20H7.33333C6.44928 20 5.60143 19.6488 4.97631 19.0237C4.35119 18.3986 4 17.5507 4 16.6667C4 15.7826 4.35119 14.9348 4.97631 14.3097C5.60143 13.6845 6.44928 13.3333 7.33333 13.3333H8"
                      stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"/>
                <path d="M20 16V24.6667C20 25.5507 19.6488 26.3986 19.0237 27.0237C18.3986 27.6488 17.5507 28 16.6667 28C15.7826 28 14.9348 27.6488 14.3097 27.0237C13.6845 26.3986 13.3333 25.5507 13.3333 24.6667V24"
                      stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"/>
              </svg>
            }
            text="Hands-on tutorials from basics to advanced VLA models"
          />
          <FeatureCard
            icon={
              <svg width="32" height="32" viewBox="0 0 32 32" fill="none">
                <rect x="4.21191" y="5.48926" width="9.42373" height="9.42373" rx="2" strokeWidth="2" stroke="currentColor"/>
                <rect x="4.21191" y="19.4453" width="9.42373" height="9.42373" rx="2" strokeWidth="2" stroke="currentColor"/>
                <rect x="18.166" y="19.4453" width="9.42373" height="9.42373" rx="2" strokeWidth="2" stroke="currentColor"/>
              </svg>
            }
            text="AI-powered chatbot for instant answers"
          />
        </div>
      </div>
    </div>
  );
}

function DetailSection() {
  const detailsData = [
    {
      icon: (
        <svg width="48" height="48" viewBox="0 0 48 48" fill="none">
          <path d="M24 8L40 16V32L24 40L8 32V16L24 8Z" stroke="#667eea" strokeWidth="2" fill="none"/>
          <circle cx="24" cy="24" r="4" fill="#764ba2"/>
          <path d="M24 12L24 20M24 28L24 36M12 18L20 22M28 26L36 30M12 30L20 26M28 22L36 18"
                stroke="#667eea" strokeWidth="2" strokeLinecap="round"/>
        </svg>
      ),
      title: "ROS 2 Integration",
      description: "Learn the Robot Operating System 2 with practical nodes, topics, services, and real robot implementations. Master the nervous system of modern robotics."
    },
    {
      icon: (
        <svg width="48" height="48" viewBox="0 0 48 48" fill="none">
          <rect x="8" y="12" width="32" height="24" rx="2" stroke="#667eea" strokeWidth="2" fill="none"/>
          <path d="M16 20L24 24L32 20M24 24V32" stroke="#764ba2" strokeWidth="2" strokeLinecap="round"/>
          <circle cx="24" cy="16" r="2" fill="#f093fb"/>
          <circle cx="16" cy="28" r="2" fill="#667eea"/>
          <circle cx="32" cy="28" r="2" fill="#667eea"/>
        </svg>
      ),
      title: "Digital Twins",
      description: "Build photorealistic simulations with Gazebo and Isaac Sim. Train AI models in virtual environments before deploying to real robots, accelerating development 10x."
    },
    {
      icon: (
        <svg width="48" height="48" viewBox="0 0 48 48" fill="none">
          <path d="M12 24C12 17.4 17.4 12 24 12C30.6 12 36 17.4 36 24" stroke="#667eea" strokeWidth="2" fill="none"/>
          <circle cx="24" cy="24" r="3" fill="#764ba2"/>
          <path d="M18 30L24 24L30 30M24 24L24 36" stroke="#667eea" strokeWidth="2" strokeLinecap="round"/>
          <rect x="20" y="34" width="8" height="4" rx="1" fill="#f093fb"/>
        </svg>
      ),
      title: "Vision-Language-Action",
      description: "Implement cutting-edge VLA models that enable robots to understand natural language, perceive their environment, and execute complex manipulation tasks."
    },
    {
      icon: (
        <svg width="48" height="48" viewBox="0 0 48 48" fill="none">
          <rect x="20" y="8" width="8" height="8" rx="2" stroke="#667eea" strokeWidth="2" fill="none"/>
          <rect x="18" y="18" width="12" height="14" rx="2" stroke="#667eea" strokeWidth="2" fill="none"/>
          <rect x="14" y="20" width="4" height="8" rx="1" stroke="#764ba2" strokeWidth="2" fill="none"/>
          <rect x="30" y="20" width="4" height="8" rx="1" stroke="#764ba2" strokeWidth="2" fill="none"/>
          <rect x="20" y="34" width="3" height="8" rx="1" fill="#f093fb"/>
          <rect x="25" y="34" width="3" height="8" rx="1" fill="#f093fb"/>
          <circle cx="22" cy="12" r="1" fill="#667eea"/>
          <circle cx="26" cy="12" r="1" fill="#667eea"/>
        </svg>
      ),
      title: "Humanoid Robotics",
      description: "Explore bipedal locomotion, manipulation, and human-robot interaction. Study real platforms like Unitree G1, Tesla Optimus, and Boston Dynamics Atlas."
    }
  ];

  return (
    <div className={styles.detailSection}>
      {detailsData.map((detail, idx) => (
        <div key={idx} className={styles.detailItem}>
          <div className={styles.detailIcon}>{detail.icon}</div>
          <span className={styles.detailTitle}>{detail.title}</span>
          <p>{detail.description}</p>
        </div>
      ))}
    </div>
  );
}

export default function Home(): JSX.Element {
  return (
    <Layout
      title="Physical AI & Humanoid Robotics Documentation"
      description="Comprehensive guide to Physical AI, humanoid robotics, ROS 2, digital twins, and vision-language-action models">
      <main className={styles.mainWrapper}>
        <HomepageHero />

        <div className={styles.cardGrid}>
          <NavigationCard
            title="Foundations"
            links={[
              { label: 'Introduction to Physical AI', href: '/docs/chapter1' },
              { label: 'Core Technologies', href: '/docs/chapter2' },
              { label: 'AI Models for Robots', href: '/docs/chapter3' },
            ]}
          />

          <NavigationCard
            title="Humanoid Robotics"
            links={[
              { label: 'Rise of Humanoids', href: '/docs/chapter4' },
              { label: 'Mechanical Design', href: '/docs/chapter5' },
              { label: 'Control & Locomotion', href: '/docs/chapter6' },
            ]}
          />

          <NavigationCard
            title="Intelligence & Autonomy"
            links={[
              { label: 'Perception Systems', href: '/docs/chapter7' },
              { label: 'Learning & Adaptation', href: '/docs/chapter8' },
              { label: 'Multimodal AI', href: '/docs/chapter9' },
            ]}
          />

          <NavigationCard
            title="Technical Modules"
            links={[
              { label: 'ROS 2 Deep Dive', href: '/docs/module1-ros2' },
              { label: 'Gazebo & Isaac Sim', href: '/docs/module2-digital-twin' },
              { label: 'Vision-Language-Action', href: '/docs/module4-vla' },
            ]}
          />

          <NavigationCard
            title="Applications & Future"
            links={[
              { label: 'Real-World Applications', href: '/docs/chapter10' },
              { label: 'Challenges & Limitations', href: '/docs/chapter11' },
              { label: 'Future of Physical AI', href: '/docs/chapter12' },
            ]}
          />

          <CTACard />
        </div>

        <WhySection />
        <DetailSection />
      </main>
    </Layout>
  );
}
