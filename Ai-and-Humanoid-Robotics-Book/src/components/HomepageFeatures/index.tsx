import type {ReactNode} from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import Heading from '@theme/Heading';
import styles from './styles.module.css';

type ChapterItem = {
  id: string;
  number: number;
  title: string;
  description: ReactNode;
  icon: string;
};

const ChapterList: ChapterItem[] = [
  {
    id: 'intro',
    number: 1,
    title: 'Introduction to Physical AI',
    description: (
      <>
        Fundamentals of Physical AI, why it matters, key technologies (perception,
        reasoning, action), and the evolution of robotics from mechanical arms to
        intelligent autonomous systems.
      </>
    ),
    icon: '🤖',
  },
  {
    id: 'ros2',
    number: 2,
    title: 'ROS 2 Fundamentals',
    description: (
      <>
        Robot Operating System 2 architecture, nodes, topics, services, actions,
        and URDF. Build distributed robot systems with DDS middleware.
      </>
    ),
    icon: '🔗',
  },
  {
    id: 'gazebo',
    number: 3,
    title: 'Gazebo Simulation',
    description: (
      <>
        Physics simulation with accurate dynamics, sensor simulation (camera, LiDAR,
        IMU), SDF format, plugins, and testing algorithms before hardware deployment.
      </>
    ),
    icon: '🌍',
  },
  {
    id: 'isaac',
    number: 4,
    title: 'NVIDIA Isaac Platform',
    description: (
      <>
        Professional robotics platform with Isaac Sim (digital twins), Isaac GEM
        (manipulation skills), motion planning, and reinforcement learning in
        photorealistic environments.
      </>
    ),
    icon: '🎮',
  },
  {
    id: 'vla',
    number: 5,
    title: 'Vision-Language-Action Models',
    description: (
      <>
        Multimodal AI for robots combining computer vision, language understanding,
        and motor control. Training, fine-tuning, and deploying VLAs on real hardware
        with Claude API integration.
      </>
    ),
    icon: '🧠',
  },
  {
    id: 'capstone',
    number: 6,
    title: 'Capstone Project',
    description: (
      <>
        End-to-end robotic system integrating perception, planning, and control.
        Hardware setup, software stack, integration tests, deployment checklist,
        and real-world implementation patterns.
      </>
    ),
    icon: '🚀',
  },
];

function Chapter({id, number, title, description, icon}: ChapterItem) {
  return (
    <div className={clsx('col col--4')}>
      <Link to={`/docs/${id}`} className={styles.chapterCard}>
        <div className="text--center padding-top--lg">
          <div style={{fontSize: '48px', marginBottom: '10px'}}>{icon}</div>
          <Heading as="h3">
            Chapter {number}
            <br />
            {title}
          </Heading>
        </div>
        <div className="text--center padding-horiz--md padding-bottom--lg">
          <p>{description}</p>
          <p style={{marginTop: '15px', fontWeight: 'bold', color: '#0ea5e9'}}>
            Read Chapter {number} →
          </p>
        </div>
      </Link>
    </div>
  );
}

export default function HomepageFeatures(): ReactNode {
  return (
    <section className={styles.features}>
      <div className="container">
        <div className="text--center margin-bottom--xl">
          <Heading as="h2">📖 Complete 6-Chapter Textbook</Heading>
          <p style={{fontSize: '18px', marginTop: '10px'}}>
            From Physical AI fundamentals to end-to-end robotic systems.
            15,000+ words with 30+ code examples.
          </p>
        </div>
        <div className="row">
          {ChapterList.map((props, idx) => (
            <Chapter key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}
