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
        {
          type: 'doc',
          id: 'intro',
          label: '📌 Chapter Overview'
        },
        {
          type: 'category',
          label: 'What is Physical AI?',
          collapsed: true,
          items: []
        },
        {
          type: 'category',
          label: 'Why Physical AI Matters',
          collapsed: true,
          items: []
        },
        {
          type: 'category',
          label: 'Historical Context',
          collapsed: true,
          items: []
        },
        {
          type: 'category',
          label: 'Key Technologies in Physical AI',
          collapsed: true,
          items: []
        },
        {
          type: 'category',
          label: 'Course Overview & Learning Outcomes',
          collapsed: true,
          items: []
        },
        {
          type: 'category',
          label: 'How to Use This Book',
          collapsed: true,
          items: []
        },
      ]
    },
    {
      type: 'category',
      label: '🤖 Chapter 2: ROS 2 Fundamentals',
      collapsed: false,
      items: [
        {
          type: 'doc',
          id: 'ros2',
          label: '📌 Chapter Overview'
        },
        {
          type: 'category',
          label: 'What is ROS 2?',
          collapsed: true,
          items: []
        },
        {
          type: 'category',
          label: 'Architecture Overview',
          collapsed: true,
          items: []
        },
        {
          type: 'category',
          label: 'Installation & Setup',
          collapsed: true,
          items: []
        },
        {
          type: 'category',
          label: 'Your First ROS 2 Program',
          collapsed: true,
          items: []
        },
        {
          type: 'category',
          label: 'Common Tools',
          collapsed: true,
          items: []
        },
        {
          type: 'category',
          label: 'Best Practices',
          collapsed: true,
          items: []
        },
        {
          type: 'category',
          label: 'Debugging & Troubleshooting',
          collapsed: true,
          items: []
        },
      ]
    },
    {
      type: 'category',
      label: '🎮 Chapter 3: Gazebo Simulation',
      collapsed: false,
      items: [
        {
          type: 'doc',
          id: 'gazebo',
          label: '📌 Chapter Overview'
        },
        {
          type: 'category',
          label: 'Introduction to Gazebo',
          collapsed: true,
          items: []
        },
        {
          type: 'category',
          label: 'Installation',
          collapsed: true,
          items: []
        },
        {
          type: 'category',
          label: 'Basic Concepts',
          collapsed: true,
          items: []
        },
        {
          type: 'category',
          label: 'Plugins for ROS 2',
          collapsed: true,
          items: []
        },
        {
          type: 'category',
          label: 'Running Simulations',
          collapsed: true,
          items: []
        },
        {
          type: 'category',
          label: 'Best Practices',
          collapsed: true,
          items: []
        },
        {
          type: 'category',
          label: 'Troubleshooting',
          collapsed: true,
          items: []
        },
      ]
    },
    {
      type: 'category',
      label: '⚙️ Chapter 4: NVIDIA Isaac Platform',
      collapsed: false,
      items: [
        {
          type: 'doc',
          id: 'isaac',
          label: '📌 Chapter Overview'
        },
        {
          type: 'category',
          label: 'Introduction to Isaac',
          collapsed: true,
          items: []
        },
        {
          type: 'category',
          label: 'Isaac Sim in Detail',
          collapsed: true,
          items: []
        },
        {
          type: 'category',
          label: 'Isaac ROS 2 Integration',
          collapsed: true,
          items: []
        },
        {
          type: 'category',
          label: 'Vision Perception Stack',
          collapsed: true,
          items: []
        },
        {
          type: 'category',
          label: 'Motion Planning',
          collapsed: true,
          items: []
        },
        {
          type: 'category',
          label: 'Best Practices',
          collapsed: true,
          items: []
        },
        {
          type: 'category',
          label: 'Real-World Deployment',
          collapsed: true,
          items: []
        },
      ]
    },
    {
      type: 'category',
      label: '🧠 Chapter 5: Vision-Language-Action Models',
      collapsed: false,
      items: [
        {
          type: 'doc',
          id: 'vla',
          label: '📌 Chapter Overview'
        },
        {
          type: 'category',
          label: 'Foundation Models for Robotics',
          collapsed: true,
          items: []
        },
        {
          type: 'category',
          label: 'VLA Architecture Deep Dive',
          collapsed: true,
          items: []
        },
        {
          type: 'category',
          label: 'Training Vision-Language-Action Models',
          collapsed: true,
          items: []
        },
        {
          type: 'category',
          label: 'Deployment and Inference',
          collapsed: true,
          items: []
        },
        {
          type: 'category',
          label: 'Real-World Examples',
          collapsed: true,
          items: []
        },
        {
          type: 'category',
          label: 'Best Practices',
          collapsed: true,
          items: []
        },
      ]
    },
    {
      type: 'category',
      label: '🏗️ Chapter 6: Capstone Project',
      collapsed: false,
      items: [
        {
          type: 'doc',
          id: 'capstone',
          label: '📌 Chapter Overview'
        },
        {
          type: 'category',
          label: 'Project Overview',
          collapsed: true,
          items: []
        },
        {
          type: 'category',
          label: 'System Architecture',
          collapsed: true,
          items: []
        },
        {
          type: 'category',
          label: 'Hardware Integration',
          collapsed: true,
          items: []
        },
        {
          type: 'category',
          label: 'Perception Pipeline',
          collapsed: true,
          items: []
        },
        {
          type: 'category',
          label: 'Planning and Control',
          collapsed: true,
          items: []
        },
        {
          type: 'category',
          label: 'Integration and Testing',
          collapsed: true,
          items: []
        },
        {
          type: 'category',
          label: 'Deployment to Hardware',
          collapsed: true,
          items: []
        },
        {
          type: 'category',
          label: 'Advanced Enhancements',
          collapsed: true,
          items: []
        },
      ]
    },
  ]
};

export default sidebars;
