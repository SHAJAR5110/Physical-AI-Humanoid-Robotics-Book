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
      label: '📖 Chapter 1: Introduction to Physical AI',
      collapsed: false,
      items: [
        'intro',
        {
          type: 'category',
          label: 'What is Physical AI?',
          collapsed: true,
          items: ['intro']
        },
        {
          type: 'category',
          label: 'Why Physical AI Matters',
          collapsed: true,
          items: ['intro']
        },
        {
          type: 'category',
          label: 'Historical Context',
          collapsed: true,
          items: ['intro']
        },
        {
          type: 'category',
          label: 'Key Technologies in Physical AI',
          collapsed: true,
          items: ['intro']
        },
        {
          type: 'category',
          label: 'Course Overview & Learning Outcomes',
          collapsed: true,
          items: ['intro']
        },
        {
          type: 'category',
          label: 'How to Use This Book',
          collapsed: true,
          items: ['intro']
        },
      ]
    },
    {
      type: 'category',
      label: '🤖 Chapter 2: ROS 2 Fundamentals',
      collapsed: false,
      items: [
        'ros2',
        {
          type: 'category',
          label: 'What is ROS 2?',
          collapsed: true,
          items: ['ros2']
        },
        {
          type: 'category',
          label: 'Architecture Overview',
          collapsed: true,
          items: ['ros2']
        },
        {
          type: 'category',
          label: 'Installation & Setup',
          collapsed: true,
          items: ['ros2']
        },
        {
          type: 'category',
          label: 'Your First ROS 2 Program',
          collapsed: true,
          items: ['ros2']
        },
        {
          type: 'category',
          label: 'Common Tools',
          collapsed: true,
          items: ['ros2']
        },
        {
          type: 'category',
          label: 'Best Practices',
          collapsed: true,
          items: ['ros2']
        },
        {
          type: 'category',
          label: 'Debugging & Troubleshooting',
          collapsed: true,
          items: ['ros2']
        },
      ]
    },
    {
      type: 'category',
      label: '🎮 Chapter 3: Gazebo Simulation',
      collapsed: false,
      items: [
        'gazebo',
        {
          type: 'category',
          label: 'Introduction to Gazebo',
          collapsed: true,
          items: ['gazebo']
        },
        {
          type: 'category',
          label: 'Installation',
          collapsed: true,
          items: ['gazebo']
        },
        {
          type: 'category',
          label: 'Basic Concepts',
          collapsed: true,
          items: ['gazebo']
        },
        {
          type: 'category',
          label: 'Plugins for ROS 2',
          collapsed: true,
          items: ['gazebo']
        },
        {
          type: 'category',
          label: 'Running Simulations',
          collapsed: true,
          items: ['gazebo']
        },
        {
          type: 'category',
          label: 'Best Practices',
          collapsed: true,
          items: ['gazebo']
        },
        {
          type: 'category',
          label: 'Troubleshooting',
          collapsed: true,
          items: ['gazebo']
        },
      ]
    },
    {
      type: 'category',
      label: '⚙️ Chapter 4: NVIDIA Isaac Platform',
      collapsed: false,
      items: [
        'isaac',
        {
          type: 'category',
          label: 'Introduction to Isaac',
          collapsed: true,
          items: ['isaac']
        },
        {
          type: 'category',
          label: 'Isaac Sim in Detail',
          collapsed: true,
          items: ['isaac']
        },
        {
          type: 'category',
          label: 'Isaac ROS 2 Integration',
          collapsed: true,
          items: ['isaac']
        },
        {
          type: 'category',
          label: 'Vision Perception Stack',
          collapsed: true,
          items: ['isaac']
        },
        {
          type: 'category',
          label: 'Motion Planning',
          collapsed: true,
          items: ['isaac']
        },
        {
          type: 'category',
          label: 'Best Practices',
          collapsed: true,
          items: ['isaac']
        },
        {
          type: 'category',
          label: 'Real-World Deployment',
          collapsed: true,
          items: ['isaac']
        },
      ]
    },
    {
      type: 'category',
      label: '🧠 Chapter 5: Vision-Language-Action Models',
      collapsed: false,
      items: [
        'vla',
        {
          type: 'category',
          label: 'Foundation Models for Robotics',
          collapsed: true,
          items: ['vla']
        },
        {
          type: 'category',
          label: 'VLA Architecture Deep Dive',
          collapsed: true,
          items: ['vla']
        },
        {
          type: 'category',
          label: 'Training Vision-Language-Action Models',
          collapsed: true,
          items: ['vla']
        },
        {
          type: 'category',
          label: 'Deployment and Inference',
          collapsed: true,
          items: ['vla']
        },
        {
          type: 'category',
          label: 'Real-World Examples',
          collapsed: true,
          items: ['vla']
        },
        {
          type: 'category',
          label: 'Best Practices',
          collapsed: true,
          items: ['vla']
        },
      ]
    },
    {
      type: 'category',
      label: '🏗️ Chapter 6: Capstone Project',
      collapsed: false,
      items: [
        'capstone',
        {
          type: 'category',
          label: 'Project Overview',
          collapsed: true,
          items: ['capstone']
        },
        {
          type: 'category',
          label: 'System Architecture',
          collapsed: true,
          items: ['capstone']
        },
        {
          type: 'category',
          label: 'Hardware Integration',
          collapsed: true,
          items: ['capstone']
        },
        {
          type: 'category',
          label: 'Perception Pipeline',
          collapsed: true,
          items: ['capstone']
        },
        {
          type: 'category',
          label: 'Planning and Control',
          collapsed: true,
          items: ['capstone']
        },
        {
          type: 'category',
          label: 'Integration and Testing',
          collapsed: true,
          items: ['capstone']
        },
        {
          type: 'category',
          label: 'Deployment to Hardware',
          collapsed: true,
          items: ['capstone']
        },
        {
          type: 'category',
          label: 'Advanced Enhancements',
          collapsed: true,
          items: ['capstone']
        },
      ]
    },
  ]
};

export default sidebars;
