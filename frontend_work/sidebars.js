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
      ],
    },
  ],
};

export default sidebars;
