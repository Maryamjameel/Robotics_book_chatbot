import type {ReactNode} from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import Heading from '@theme/Heading';
import {motion} from 'framer-motion';
import {useInView} from 'framer-motion';
import {useRef} from 'react';

import styles from './index.module.css';

// Robotic-themed SVG icons
function RobotIcon() {
  return (
    <svg viewBox="0 0 64 64" fill="none" xmlns="http://www.w3.org/2000/svg" className={styles.featureIcon}>
      <rect x="16" y="24" width="32" height="28" fill="currentColor" fillOpacity="0.1" stroke="currentColor" strokeWidth="2" rx="4"/>
      <circle cx="26" cy="34" r="3" fill="currentColor"/>
      <circle cx="38" cy="34" r="3" fill="currentColor"/>
      <rect x="28" y="14" width="8" height="10" fill="currentColor" fillOpacity="0.1" stroke="currentColor" strokeWidth="2" rx="2"/>
      <circle cx="32" cy="10" r="3" fill="currentColor"/>
      <line x1="16" y1="44" x2="10" y2="44" stroke="currentColor" strokeWidth="2" strokeLinecap="round"/>
      <line x1="48" y1="44" x2="54" y2="44" stroke="currentColor" strokeWidth="2" strokeLinecap="round"/>
      <rect x="24" y="42" width="16" height="4" fill="currentColor" fillOpacity="0.2" rx="2"/>
    </svg>
  );
}

function CodeIcon() {
  return (
    <svg viewBox="0 0 64 64" fill="none" xmlns="http://www.w3.org/2000/svg" className={styles.featureIcon}>
      <polyline points="20,18 10,32 20,46" stroke="currentColor" strokeWidth="3" strokeLinecap="round" strokeLinejoin="round" fill="none"/>
      <polyline points="44,18 54,32 44,46" stroke="currentColor" strokeWidth="3" strokeLinecap="round" strokeLinejoin="round" fill="none"/>
      <line x1="36" y1="14" x2="28" y2="50" stroke="currentColor" strokeWidth="3" strokeLinecap="round"/>
    </svg>
  );
}

function GearIcon() {
  return (
    <svg viewBox="0 0 64 64" fill="none" xmlns="http://www.w3.org/2000/svg" className={styles.featureIcon}>
      <circle cx="32" cy="32" r="8" fill="currentColor" fillOpacity="0.2" stroke="currentColor" strokeWidth="2"/>
      <path d="M32,10 L36,18 L32,26 L28,18 Z" fill="currentColor" fillOpacity="0.1" stroke="currentColor" strokeWidth="1.5"/>
      <path d="M54,32 L46,36 L38,32 L46,28 Z" fill="currentColor" fillOpacity="0.1" stroke="currentColor" strokeWidth="1.5"/>
      <path d="M32,54 L28,46 L32,38 L36,46 Z" fill="currentColor" fillOpacity="0.1" stroke="currentColor" strokeWidth="1.5"/>
      <path d="M10,32 L18,28 L26,32 L18,36 Z" fill="currentColor" fillOpacity="0.1" stroke="currentColor" strokeWidth="1.5"/>
    </svg>
  );
}

function BookIcon() {
  return (
    <svg viewBox="0 0 64 64" fill="none" xmlns="http://www.w3.org/2000/svg" className={styles.featureIcon}>
      <rect x="14" y="10" width="36" height="44" fill="currentColor" fillOpacity="0.05" stroke="currentColor" strokeWidth="2" rx="2"/>
      <line x1="14" y1="32" x2="32" y2="32" stroke="currentColor" strokeWidth="2"/>
      <path d="M32,10 L32,54 Q32,58 36,58 Q40,58 40,54 L40,10" fill="currentColor" fillOpacity="0.1" stroke="currentColor" strokeWidth="2"/>
      <line x1="20" y1="20" x2="28" y2="20" stroke="currentColor" strokeWidth="1.5" strokeLinecap="round"/>
      <line x1="20" y1="25" x2="28" y2="25" stroke="currentColor" strokeWidth="1.5" strokeLinecap="round"/>
      <line x1="20" y1="38" x2="28" y2="38" stroke="currentColor" strokeWidth="1.5" strokeLinecap="round"/>
      <line x1="20" y1="43" x2="28" y2="43" stroke="currentColor" strokeWidth="1.5" strokeLinecap="round"/>
    </svg>
  );
}

interface FeatureItem {
  title: string;
  icon: ReactNode;
  description: ReactNode;
}

const FeatureList: FeatureItem[] = [
  {
    title: 'Comprehensive Robotics Education',
    icon: <RobotIcon />,
    description: (
      <>
        Master humanoid robotics from fundamentals to advanced topics. Learn inverse kinematics,
        motion planning, sensor fusion, and control systems with hands-on examples.
      </>
    ),
  },
  {
    title: 'ROS 2 & Modern Frameworks',
    icon: <CodeIcon />,
    description: (
      <>
        Deep dive into ROS 2 architecture, Python/C++ programming, and cutting-edge frameworks.
        Build real-world robotics applications with industry-standard tools.
      </>
    ),
  },
  {
    title: 'Hardware & Simulation',
    icon: <GearIcon />,
    description: (
      <>
        Understand actuators, sensors, and embedded systems. Practice with Gazebo simulation
        before deploying to real hardware platforms.
      </>
    ),
  },
  {
    title: 'Interactive Learning',
    icon: <BookIcon />,
    description: (
      <>
        Follow structured lessons with code examples, exercises, and projects. Access a searchable
        glossary and chapter navigation for efficient learning.
      </>
    ),
  },
];

function Feature({title, icon, description, index}: FeatureItem & {index: number}) {
  const ref = useRef(null);
  const isInView = useInView(ref, {once: true, margin: "-100px"});

  return (
    <motion.div
      ref={ref}
      className={clsx('col col--3', styles.feature)}
      initial={{opacity: 0, y: 50}}
      animate={isInView ? {opacity: 1, y: 0} : {opacity: 0, y: 50}}
      transition={{duration: 0.6, delay: index * 0.1}}
      whileHover={{scale: 1.02}}
    >
      <div className={styles.featureCard}>
        <motion.div
          className={styles.featureIconWrapper}
          whileHover={{rotate: 360}}
          transition={{duration: 0.6}}
        >
          {icon}
        </motion.div>
        <Heading as="h3" className={styles.featureTitle}>{title}</Heading>
        <p className={styles.featureDescription}>{description}</p>
      </div>
    </motion.div>
  );
}

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={styles.heroBanner}>
      <div className={styles.heroBackground}></div>
      <div className="container">
        <div className={styles.heroContent}>
          <motion.div
            className={styles.heroTextContent}
            initial={{opacity: 0, x: -50}}
            animate={{opacity: 1, x: 0}}
            transition={{duration: 0.8}}
          >
            <motion.div
              className={styles.heroTag}
              initial={{opacity: 0, y: -20}}
              animate={{opacity: 1, y: 0}}
              transition={{duration: 0.6, delay: 0.2}}
            >
              Physical AI & Advanced Robotics
            </motion.div>
            <motion.div
              initial={{opacity: 0, y: 30}}
              animate={{opacity: 1, y: 0}}
              transition={{duration: 0.8, delay: 0.3}}
            >
              <Heading as="h1" className={styles.heroTitle}>
                {siteConfig.title}
              </Heading>
            </motion.div>
            <motion.p
              className={styles.heroSubtitle}
              initial={{opacity: 0, y: 30}}
              animate={{opacity: 1, y: 0}}
              transition={{duration: 0.8, delay: 0.4}}
            >
              {siteConfig.tagline}
            </motion.p>
            <motion.p
              className={styles.heroDescription}
              initial={{opacity: 0, y: 30}}
              animate={{opacity: 1, y: 0}}
              transition={{duration: 0.8, delay: 0.5}}
            >
              A comprehensive guide to humanoid robotics, ROS 2, and intelligent control systems.
              From kinematics to machine learning—build the future of robotics.
            </motion.p>
            <motion.div
              className={styles.heroButtons}
              initial={{opacity: 0, y: 30}}
              animate={{opacity: 1, y: 0}}
              transition={{duration: 0.8, delay: 0.6}}
            >
              <motion.div
                whileHover={{scale: 1.05}}
                whileTap={{scale: 0.95}}
              >
                <Link
                  className={clsx('button button--primary button--lg', styles.primaryButton)}
                  to="/docs/chapters/chapter-01/lesson-01-foundations">
                  Start Learning
                  <span className={styles.buttonArrow}>→</span>
                </Link>
              </motion.div>
              <motion.div
                whileHover={{scale: 1.05}}
                whileTap={{scale: 0.95}}
              >
                <Link
                  className={clsx('button button--outline button--lg', styles.secondaryButton)}
                  to="/docs/glossary">
                  Browse Glossary
                </Link>
              </motion.div>
            </motion.div>
            <motion.div
              className={styles.heroStats}
              initial={{opacity: 0, y: 30}}
              animate={{opacity: 1, y: 0}}
              transition={{duration: 0.8, delay: 0.7}}
            >
              <motion.div
                className={styles.stat}
                whileHover={{scale: 1.1}}
                transition={{duration: 0.3}}
              >
                <div className={styles.statNumber}>75+</div>
                <div className={styles.statLabel}>Technical Terms</div>
              </motion.div>
              <motion.div
                className={styles.stat}
                whileHover={{scale: 1.1}}
                transition={{duration: 0.3}}
              >
                <div className={styles.statNumber}>10+</div>
                <div className={styles.statLabel}>Chapters</div>
              </motion.div>
              <motion.div
                className={styles.stat}
                whileHover={{scale: 1.1}}
                transition={{duration: 0.3}}
              >
                <div className={styles.statNumber}>100%</div>
                <div className={styles.statLabel}>Open Source</div>
              </motion.div>
            </motion.div>
          </motion.div>
          <motion.div
            className={styles.heroImageWrapper}
            initial={{opacity: 0, x: 50, rotate: -10}}
            animate={{opacity: 1, x: 0, rotate: 0}}
            transition={{duration: 1, delay: 0.4}}
          >
            <motion.img
              src="img/pexels-kindelmedia-8566521.jpg"
              alt="Advanced Humanoid Robot"
              className={styles.heroImage}
              animate={{
                y: [0, -20, 0],
              }}
              transition={{
                duration: 4,
                repeat: Infinity,
                ease: "easeInOut"
              }}
            />
            <div className={styles.heroImageGlow}></div>
          </motion.div>
        </div>
      </div>
    </header>
  );
}

function HomepageFeatures() {
  const ref = useRef(null);
  const isInView = useInView(ref, {once: true, margin: "-100px"});

  return (
    <section ref={ref} className={styles.features}>
      <div className="container">
        <motion.div
          className={styles.featuresHeader}
          initial={{opacity: 0, y: 30}}
          animate={isInView ? {opacity: 1, y: 0} : {opacity: 0, y: 30}}
          transition={{duration: 0.6}}
        >
          <Heading as="h2" className={styles.sectionTitle}>
            What You'll Master
          </Heading>
          <p className={styles.sectionSubtitle}>
            Everything you need to become a robotics engineer
          </p>
        </motion.div>
        <div className="row">
          {FeatureList.map((props, idx) => (
            <Feature key={idx} {...props} index={idx} />
          ))}
        </div>
      </div>
    </section>
  );
}

function TopicsSection() {
  const topics = [
    'Forward & Inverse Kinematics',
    'ROS 2 Architecture',
    'Motion Planning',
    'Sensor Fusion',
    'PID Control',
    'Computer Vision',
    'Machine Learning',
    'Hardware Integration',
  ];

  const ref = useRef(null);
  const isInView = useInView(ref, {once: true, margin: "-100px"});

  return (
    <section ref={ref} className={styles.topicsSection}>
      <div className="container">
        <motion.div
          initial={{opacity: 0, y: 30}}
          animate={isInView ? {opacity: 1, y: 0} : {opacity: 0, y: 30}}
          transition={{duration: 0.6}}
        >
          <Heading as="h2" className={styles.sectionTitle}>
            Core Topics Covered
          </Heading>
        </motion.div>
        <div className={styles.topicsGrid}>
          {topics.map((topic, idx) => (
            <motion.div
              key={idx}
              className={styles.topicChip}
              initial={{opacity: 0, x: -30}}
              animate={isInView ? {opacity: 1, x: 0} : {opacity: 0, x: -30}}
              transition={{duration: 0.5, delay: idx * 0.08}}
              whileHover={{
                x: 8,
                borderColor: 'rgba(0, 217, 255, 0.6)',
                boxShadow: '0 4px 20px rgba(0, 217, 255, 0.3)'
              }}
            >
              <motion.span
                className={styles.topicDot}
                animate={{
                  scale: [1, 1.2, 1],
                }}
                transition={{
                  duration: 2,
                  repeat: Infinity,
                  delay: idx * 0.2
                }}
              >
                ●
              </motion.span>
              {topic}
            </motion.div>
          ))}
        </div>
      </div>
    </section>
  );
}

function ApplicationsSection() {
  const applications = [
    {
      title: 'Humanoid Navigation',
      description: 'Implement advanced navigation algorithms for complex environments.',
      image: 'img/pexels-cookiecutter-1148820.jpg',
    },
    {
      title: 'Industrial Automation',
      description: 'Develop robotic systems for manufacturing and industrial processes.',
      image: 'img/pexels-cottonbro-6153741.jpg',
    },
    {
      title: 'Service Robotics',
      description: 'Design and program robots for assistance in various service industries.',
      image: 'img/image-from-rawpixel-id-18501930-jpeg.jpg',
    },
  ];

  const ref = useRef(null);
  const isInView = useInView(ref, {once: true, margin: "-100px"});

  return (
    <section ref={ref} className={styles.applicationsSection}>
      <div className="container">
        <motion.div
          initial={{opacity: 0, y: 30}}
          animate={isInView ? {opacity: 1, y: 0} : {opacity: 0, y: 30}}
          transition={{duration: 0.6}}
        >
          <Heading as="h2" className={styles.sectionTitle}>
            Real-World Applications
          </Heading>
          <p className={styles.sectionSubtitle}>
            Apply your knowledge to practical and impactful robotics projects.
          </p>
        </motion.div>
        <div className={styles.applicationsGrid}>
          {applications.map((app, idx) => (
            <motion.div
              key={idx}
              className={styles.applicationCard}
              initial={{opacity: 0, y: 50}}
              animate={isInView ? {opacity: 1, y: 0} : {opacity: 0, y: 50}}
              transition={{duration: 0.6, delay: idx * 0.15}}
              whileHover={{
                y: -12,
                boxShadow: '0 20px 60px rgba(0, 217, 255, 0.3)'
              }}
            >
              <div className={styles.applicationImageWrapper}>
                <motion.img
                  src={app.image}
                  alt={app.title}
                  className={styles.applicationImage}
                  whileHover={{scale: 1.1}}
                  transition={{duration: 0.4}}
                />
              </div>
              <div className={styles.applicationContent}>
                <Heading as="h3" className={styles.applicationTitle}>{app.title}</Heading>
                <p className={styles.applicationDescription}>{app.description}</p>
              </div>
            </motion.div>
          ))}
        </div>
      </div>
    </section>
  );
}

function TestimonialsSection() {
  const testimonials = [
    {
      quote: "This textbook transformed my understanding of robotics. The practical examples and clear explanations are outstanding!",
      author: "Dr. Anya Sharma",
      role: "Robotics Researcher at MIT",
      avatar: "img/pexels-kindelmedia-8566467.jpg",
    },
    {
      quote: "A must-read for anyone serious about humanoid robotics. It covers everything from theory to implementation with ROS 2.",
      author: "Prof. Ben Carter",
      role: "Lead Engineer at Boston Dynamics",
      avatar: "img/pexels-pixabay-373543.jpg",
    },
    {
      quote: "The best resource I've found for learning physical AI. Highly recommend for students and professionals alike.",
      author: "Sarah Lee",
      role: "Robotics Software Engineer",
      avatar: "img/pexels-tara-winstead-8386440.jpg",
    },
  ];

  const ref = useRef(null);
  const isInView = useInView(ref, {once: true, margin: "-100px"});

  return (
    <section ref={ref} className={styles.testimonialsSection}>
      <div className="container">
        <motion.div
          initial={{opacity: 0, y: 30}}
          animate={isInView ? {opacity: 1, y: 0} : {opacity: 0, y: 30}}
          transition={{duration: 0.6}}
        >
          <Heading as="h2" className={styles.sectionTitle}>
            What Our Readers Say
          </Heading>
          <p className={styles.sectionSubtitle}>
            Hear from students and experts who have benefited from this comprehensive guide.
          </p>
        </motion.div>
        <div className={styles.testimonialsGrid}>
          {testimonials.map((testimonial, idx) => (
            <motion.div
              key={idx}
              className={styles.testimonialCard}
              initial={{opacity: 0, scale: 0.9}}
              animate={isInView ? {opacity: 1, scale: 1} : {opacity: 0, scale: 0.9}}
              transition={{duration: 0.5, delay: idx * 0.12}}
              whileHover={{
                y: -10,
                boxShadow: '0 20px 60px rgba(0, 217, 255, 0.3)'
              }}
            >
              <p className={styles.testimonialQuote}>"{testimonial.quote}"</p>
              <div className={styles.testimonialAuthor}>
                <motion.img
                  src={testimonial.avatar}
                  alt={testimonial.author}
                  className={styles.authorAvatar}
                  whileHover={{scale: 1.1, rotate: 5}}
                  transition={{duration: 0.3}}
                />
                <div>
                  <div className={styles.authorName}>{testimonial.author}</div>
                  <div className={styles.authorRole}>{testimonial.role}</div>
                </div>
              </div>
            </motion.div>
          ))}
        </div>
      </div>
    </section>
  );
}

export default function Home(): ReactNode {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title="Home"
      description="Comprehensive textbook on humanoid robotics, ROS 2, and physical AI systems">
      <motion.div
        initial={{opacity: 0}}
        animate={{opacity: 1}}
        transition={{duration: 0.5}}
      >
        <HomepageHeader />
        <main>
          <HomepageFeatures />
          <TopicsSection />
          <ApplicationsSection />
          <TestimonialsSection />
        </main>
      </motion.div>
    </Layout>
  );
}
