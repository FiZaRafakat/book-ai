---
sidebar_position: 1
---

# AI Robotics Learning Modules

Welcome to the **AI Robotics Learning Platform** - a comprehensive educational resource for understanding robotics concepts with a focus on AI integration. This platform contains multiple modules designed to teach robotics from the ground up, with special emphasis on connecting AI logic to physical robot control.

## Available Modules

### ROS 2 Robotics Module
Learn the fundamentals of ROS 2 (Robot Operating System 2) and how it serves as middleware connecting AI systems to humanoid robot control. This module covers:

- ROS 2 communication patterns (Nodes, Topics, Services, Actions)
- Python control with rclpy
- Humanoid robot structure with URDF (Unified Robot Description Format)

### Digital Twin Module (Gazebo & Unity)
Explore digital twin technology for humanoid robots using physics simulation and visual environments. This module covers:

- Physics simulation with Gazebo
- Visual environments with Unity
- Sensor simulation for AI training

### AI-Robot Brain (NVIDIA Isaac™)
Learn advanced perception, navigation, and training techniques for humanoid robots using NVIDIA Isaac technologies. This module covers:

- Isaac Sim for photorealistic simulation and synthetic data generation
- Isaac ROS and hardware-accelerated Visual SLAM
- Path planning with Navigation2 (Nav2) for bipedal humanoid navigation

## Getting Started

Choose a module from the sidebar to begin your robotics education journey. Each module is designed to build your understanding progressively, starting with fundamental concepts and advancing to complex implementations.

### What you'll need

- [Node.js](https://nodejs.org/en/download/) version 20.0 or above:
  - When installing Node.js, you are recommended to check all checkboxes related to dependencies.

## Generate a new site

Generate a new Docusaurus site using the **classic template**.

The classic template will automatically be added to your project after you run the command:

```bash
npm init docusaurus@latest my-website classic
```

You can type this command into Command Prompt, Powershell, Terminal, or any other integrated terminal of your code editor.

The command also installs all necessary dependencies you need to run Docusaurus.

## Start your site

Run the development server:

```bash
cd my-website
npm run start
```

The `cd` command changes the directory you're working with. In order to work with your newly created Docusaurus site, you'll need to navigate the terminal there.

The `npm run start` command builds your website locally and serves it through a development server, ready for you to view at http://localhost:3000/.

Open `docs/intro.md` (this page) and edit some lines: the site **reloads automatically** and displays your changes.
