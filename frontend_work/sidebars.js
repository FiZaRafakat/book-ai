// @ts-check

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.

 @type {import('@docusaurus/plugin-content-docs').SidebarsConfig}
 */
const sidebars = {
  tutorialSidebar: [
    'intro',
    {
      type: 'category',
      label: 'ROS 2 Robotics Module',
      items: [
        'ros2-robotics/chapter-1-basics',
        'ros2-robotics/chapter-2-python-control',
        'ros2-robotics/chapter-3-urdf',
        'ros2-robotics/summary-and-next-steps',
      ],
    },
    {
      type: 'category',
      label: 'Digital Twin Module (Gazebo & Unity)',
      items: [
        'digital-twin/chapter-1-physics-simulation',
        'digital-twin/chapter-2-visual-environments',
        'digital-twin/chapter-3-sensor-simulation',
        'digital-twin/summary-and-next-steps',
      ],
    },
    {
      type: 'category',
      label: 'AI-Robot Brain (NVIDIA Isaac™)',
      items: [
        'isaac-ai-brain/chapter-1-isaac-sim',
        'isaac-ai-brain/chapter-2-isaac-ros-vslam',
        'isaac-ai-brain/chapter-3-nav2-path-planning',
        'isaac-ai-brain/summary-and-next-steps',
      ],
    },
    {
      type: 'category',
      label: 'Vision-Language-Action (VLA) Robotics',
      items: [
        'vla-integration/chapter-1-voice-to-action',
        'vla-integration/chapter-2-cognitive-planning',
        'vla-integration/chapter-3-autonomous-humanoid',
        'vla-integration/summary-and-next-steps',
      ],
    },
  ],
};

export default sidebars;
