import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

/**
 * Sidebar configuration for Physical AI & Humanoid Robotics Book
 * Includes all 5 modules (0-4) with complete chapter structure
 */
const sidebars: SidebarsConfig = {
  defaultSidebar: [
    {
      type: 'category',
      label: 'Introduction',
      items: ['intro'],
    },
    {
      type: 'category',
      label: 'Module 0: Foundations of Physical AI',
      items: [
        'module-0/intro',
        'module-0/chapter-0-1',
        'module-0/chapter-0-2',
        'module-0/chapter-0-3',
      ],
    },
    {
      type: 'category',
      label: 'Module 1: The Robotic Nervous System (ROS 2)',
      items: [
        'module-1/intro',
        'module-1/chapter-1-1',
        'module-1/chapter-1-2',
        'module-1/chapter-1-3',
        'module-1/chapter-1-4',
        'module-1/chapter-1-5',
      ],
    },
    {
      type: 'category',
      label: 'Module 2: The Digital Twin (Gazebo & Unity)',
      items: [
        'module-2/intro',
        'module-2/chapter-2-1',
        'module-2/chapter-2-2',
        'module-2/chapter-2-3',
        'module-2/chapter-2-4',
        'module-2/chapter-2-5',
      ],
    },
    {
      type: 'category',
      label: 'Module 3: The AI-Robot Brain (NVIDIA Isaac)',
      items: [
        'module-3/intro',
        'module-3/chapter-3-1',
        'module-3/chapter-3-2',
        'module-3/chapter-3-3',
        'module-3/chapter-3-4',
        'module-3/chapter-3-5',
      ],
    },
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action (VLA)',
      items: [
        'module-4/intro',
        'module-4/chapter-4-1',
        'module-4/chapter-4-2',
        'module-4/chapter-4-3',
        'module-4/chapter-4-4',
        'module-4/chapter-4-5',
        'module-4/capstone',
      ],
    },
  ],
};

export default sidebars;
