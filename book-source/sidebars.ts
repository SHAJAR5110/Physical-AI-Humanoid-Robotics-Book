import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.
 */
const sidebars: SidebarsConfig = {
  // Manually configured sidebar with chapter structure and modules
  tutorialSidebar: [
    {
      type: 'category',
      label: 'Chapter 1: Introduction to Physical AI',
      collapsed: true,
      items: [
        'intro',
        {
          type: 'link',
          label: 'What is Physical AI?',
          href: '/docs/intro#what-is-physical-ai'
        },
        {
          type: 'link',
          label: 'Why Physical AI Matters',
          href: '/docs/intro#why-physical-ai-matters'
        },
        {
          type: 'link',
          label: 'Historical Context',
          href: '/docs/intro#historical-context'
        },
        {
          type: 'link',
          label: 'Key Technologies in Physical AI',
          href: '/docs/intro#key-technologies-in-physical-ai'
        },
        {
          type: 'link',
          label: 'Course Overview',
          href: '/docs/intro#course-overview'
        },
        {
          type: 'link',
          label: 'How to Use This Book',
          href: '/docs/intro#how-to-use-this-book'
        },
      ]
    },
    {
      type: 'category',
      label: 'Chapter 2: ROS 2 Fundamentals',
      collapsed: true,
      items: [
        'ros2',
        {
          type: 'link',
          label: 'What is ROS 2?',
          href: '/docs/ros2#what-is-ros-2'
        },
        {
          type: 'link',
          label: 'Architecture Overview',
          href: '/docs/ros2#architecture-overview'
        },
        {
          type: 'link',
          label: 'Installation & Setup',
          href: '/docs/ros2#installation--setup'
        },
        {
          type: 'link',
          label: 'Your First ROS 2 Program',
          href: '/docs/ros2#your-first-ros-2-program'
        },
        {
          type: 'link',
          label: 'Common Tools',
          href: '/docs/ros2#common-tools'
        },
        {
          type: 'link',
          label: 'Best Practices',
          href: '/docs/ros2#best-practices'
        },
        {
          type: 'link',
          label: 'Debugging & Troubleshooting',
          href: '/docs/ros2#debugging--troubleshooting'
        },
      ]
    },
    {
      type: 'category',
      label: 'Chapter 3: Gazebo Simulation',
      collapsed: true,
      items: [
        'gazebo',
        {
          type: 'link',
          label: 'Introduction to Gazebo',
          href: '/docs/gazebo#introduction-to-gazebo'
        },
        {
          type: 'link',
          label: 'Installation',
          href: '/docs/gazebo#installation'
        },
        {
          type: 'link',
          label: 'Basic Concepts',
          href: '/docs/gazebo#basic-concepts'
        },
        {
          type: 'link',
          label: 'Plugins for ROS 2',
          href: '/docs/gazebo#plugins-for-ros-2'
        },
        {
          type: 'link',
          label: 'Running Simulations',
          href: '/docs/gazebo#running-simulations'
        },
        {
          type: 'link',
          label: 'Best Practices for Effective Simulation',
          href: '/docs/gazebo#best-practices-for-effective-simulation'
        },
        {
          type: 'link',
          label: 'Troubleshooting',
          href: '/docs/gazebo#troubleshooting'
        },
      ]
    },
    {
      type: 'category',
      label: 'Chapter 4: NVIDIA Isaac Platform',
      collapsed: true,
      items: [
        'isaac',
        {
          type: 'link',
          label: 'Introduction to Isaac',
          href: '/docs/isaac#introduction-to-isaac'
        },
        {
          type: 'link',
          label: 'Isaac Sim in Detail',
          href: '/docs/isaac#isaac-sim-in-detail'
        },
        {
          type: 'link',
          label: 'Isaac ROS 2 Integration in Detail',
          href: '/docs/isaac#isaac-ros-2-integration-in-detail'
        },
        {
          type: 'link',
          label: 'Testing Algorithms in Simulation',
          href: '/docs/isaac#testing-algorithms-in-simulation'
        },
        {
          type: 'link',
          label: 'Best Practices for Production Isaac Development',
          href: '/docs/isaac#best-practices-for-production-isaac-development'
        },
        {
          type: 'link',
          label: 'Bridging Simulation to ROS 2 Hardware',
          href: '/docs/isaac#bridging-simulation-to-ros-2-hardware'
        },
        {
          type: 'link',
          label: 'Real-World Deployment Example',
          href: '/docs/isaac#real-world-deployment-example'
        },
      ]
    },
    {
      type: 'category',
      label: 'Chapter 5: Vision-Language-Action Models',
      collapsed: true,
      items: [
        'vla',
        {
          type: 'link',
          label: 'Foundation Models for Robotics',
          href: '/docs/vla#foundation-models-for-robotics'
        },
        {
          type: 'link',
          label: 'VLA Architecture Deep Dive',
          href: '/docs/vla#vla-architecture-deep-dive'
        },
        {
          type: 'link',
          label: 'Training Vision-Language-Action Models',
          href: '/docs/vla#training-vision-language-action-models'
        },
        {
          type: 'link',
          label: 'Deployment and Inference',
          href: '/docs/vla#deployment-and-inference'
        },
        {
          type: 'link',
          label: 'Real-World Examples and Case Studies',
          href: '/docs/vla#real-world-examples-and-case-studies'
        },
        {
          type: 'link',
          label: 'Best Practices and Advanced Techniques',
          href: '/docs/vla#best-practices-and-advanced-techniques'
        },
      ]
    },
    {
      type: 'category',
      label: 'Chapter 6: Capstone Project',
      collapsed: true,
      items: [
        'capstone',
        {
          type: 'link',
          label: 'System Architecture Deep Dive',
          href: '/docs/capstone#system-architecture-deep-dive'
        },
        {
          type: 'link',
          label: 'System Components',
          href: '/docs/capstone#system-components'
        },
        {
          type: 'link',
          label: 'Implementation Guide',
          href: '/docs/capstone#implementation-guide'
        },
        {
          type: 'link',
          label: 'Hardware Integration',
          href: '/docs/capstone#hardware-integration'
        },
        {
          type: 'link',
          label: 'Perception Pipeline Implementation',
          href: '/docs/capstone#perception-pipeline-implementation'
        },
        {
          type: 'link',
          label: 'Planning and Control',
          href: '/docs/capstone#planning-and-control'
        },
        {
          type: 'link',
          label: 'Integration and Testing',
          href: '/docs/capstone#integration-and-testing'
        },
        {
          type: 'link',
          label: 'Deployment to Hardware',
          href: '/docs/capstone#deployment-to-hardware'
        },
        {
          type: 'link',
          label: 'Advanced Enhancements',
          href: '/docs/capstone#advanced-enhancements'
        },
      ]
    },
  ]
};

export default sidebars;
