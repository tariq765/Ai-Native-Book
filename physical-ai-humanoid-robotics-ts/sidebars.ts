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
  // Manual sidebar configuration for the Physical AI & Humanoid Robotics textbook
  textbookSidebar: [
    {
      type: 'category',
      label: 'Introduction',
      items: ['intro', 'quarter-overview'],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Module 1: ROS 2',
      items: [
        'module-1-ros2/intro',
        'module-1-ros2/ros2-basics',
        'module-1-ros2/nodes-topics-services',
        'module-1-ros2/urdf-humanoids'
      ],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Module 2: Digital Twin',
      items: [
        'module-2-digital-twin/intro',
        'module-2-digital-twin/gazebo-simulation',
        'module-2-digital-twin/unity-hri',
        'module-2-digital-twin/sensors'
      ],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Module 3: AI Robot Brain',
      items: [
        'module-3-isaac/intro',
        'module-3-isaac/isaac-sim',
        'module-3-isaac/isaac-ros',
        'module-3-isaac/nav2'
      ],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action',
      items: [
        'module-4-vla/intro',
        'module-4-vla/voice-to-action',
        'module-4-vla/llm-planning',
        'module-4-vla/capstone'
      ],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Appendices',
      items: [
        'appendices/mathematical-foundations',
        'appendices/robotics-software-tools',
        'appendices/project-ideas'
      ],
      collapsed: true,
    }
  ],
};

export default sidebars;
