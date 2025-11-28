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
  // By default, Docusaurus generates a sidebar from the docs folder structure
  defaultSidebar: [
    {
      type: 'category',
      label: 'Introduction',
      items: ['intro'],
    },
    {
      type: 'category',
      label: 'Module 1: Foundations of Physical AI',
      items: [
        'module-1/chapter-1-1',
        'module-1/chapter-1-2',
        'module-1/chapter-1-3',
        'module-1/chapter-1-4',
      ],
    },
    {
      type: 'category',
      label: 'Module 2: Computer Vision for Robotics',
      items: [
        'module-2/intro',
        'module-2/chapter-2-1',
        'module-2/chapter-2-2',
        'module-2/chapter-2-3',
        'module-2/chapter-2-4',
      ],
    },
  ],

  // But you can create a sidebar manually
  /*
  tutorialSidebar: [
    'intro',
    'hello',
    {
      type: 'category',
      label: 'Tutorial',
      items: ['tutorial-basics/create-a-document'],
    },
  ],
   */
};

export default sidebars;
