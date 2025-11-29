import React from 'react';
import ComponentCreator from '@docusaurus/ComponentCreator';

export default [
  {
    path: '/physical-ai-humanoid-robotics-book/markdown-page',
    component: ComponentCreator('/physical-ai-humanoid-robotics-book/markdown-page', '8ab'),
    exact: true
  },
  {
    path: '/physical-ai-humanoid-robotics-book/docs',
    component: ComponentCreator('/physical-ai-humanoid-robotics-book/docs', '149'),
    routes: [
      {
        path: '/physical-ai-humanoid-robotics-book/docs',
        component: ComponentCreator('/physical-ai-humanoid-robotics-book/docs', '48e'),
        routes: [
          {
            path: '/physical-ai-humanoid-robotics-book/docs',
            component: ComponentCreator('/physical-ai-humanoid-robotics-book/docs', '197'),
            routes: [
              {
                path: '/physical-ai-humanoid-robotics-book/docs/intro',
                component: ComponentCreator('/physical-ai-humanoid-robotics-book/docs/intro', '412'),
                exact: true,
                sidebar: "defaultSidebar"
              },
              {
                path: '/physical-ai-humanoid-robotics-book/docs/module-1/chapter-1-1',
                component: ComponentCreator('/physical-ai-humanoid-robotics-book/docs/module-1/chapter-1-1', '10a'),
                exact: true,
                sidebar: "defaultSidebar"
              },
              {
                path: '/physical-ai-humanoid-robotics-book/docs/module-1/chapter-1-2',
                component: ComponentCreator('/physical-ai-humanoid-robotics-book/docs/module-1/chapter-1-2', '1f2'),
                exact: true,
                sidebar: "defaultSidebar"
              },
              {
                path: '/physical-ai-humanoid-robotics-book/docs/module-1/chapter-1-3',
                component: ComponentCreator('/physical-ai-humanoid-robotics-book/docs/module-1/chapter-1-3', 'ffa'),
                exact: true,
                sidebar: "defaultSidebar"
              },
              {
                path: '/physical-ai-humanoid-robotics-book/docs/module-1/chapter-1-4',
                component: ComponentCreator('/physical-ai-humanoid-robotics-book/docs/module-1/chapter-1-4', '7da'),
                exact: true,
                sidebar: "defaultSidebar"
              },
              {
                path: '/physical-ai-humanoid-robotics-book/docs/module-2/chapter-2-1',
                component: ComponentCreator('/physical-ai-humanoid-robotics-book/docs/module-2/chapter-2-1', '262'),
                exact: true,
                sidebar: "defaultSidebar"
              },
              {
                path: '/physical-ai-humanoid-robotics-book/docs/module-2/chapter-2-2',
                component: ComponentCreator('/physical-ai-humanoid-robotics-book/docs/module-2/chapter-2-2', '38b'),
                exact: true,
                sidebar: "defaultSidebar"
              },
              {
                path: '/physical-ai-humanoid-robotics-book/docs/module-2/chapter-2-3',
                component: ComponentCreator('/physical-ai-humanoid-robotics-book/docs/module-2/chapter-2-3', '96d'),
                exact: true,
                sidebar: "defaultSidebar"
              },
              {
                path: '/physical-ai-humanoid-robotics-book/docs/module-2/chapter-2-4',
                component: ComponentCreator('/physical-ai-humanoid-robotics-book/docs/module-2/chapter-2-4', 'e4e'),
                exact: true,
                sidebar: "defaultSidebar"
              },
              {
                path: '/physical-ai-humanoid-robotics-book/docs/module-2/intro',
                component: ComponentCreator('/physical-ai-humanoid-robotics-book/docs/module-2/intro', '1cb'),
                exact: true,
                sidebar: "defaultSidebar"
              },
              {
                path: '/physical-ai-humanoid-robotics-book/docs/tutorial-basics/congratulations',
                component: ComponentCreator('/physical-ai-humanoid-robotics-book/docs/tutorial-basics/congratulations', 'a52'),
                exact: true
              },
              {
                path: '/physical-ai-humanoid-robotics-book/docs/tutorial-basics/create-a-blog-post',
                component: ComponentCreator('/physical-ai-humanoid-robotics-book/docs/tutorial-basics/create-a-blog-post', '5e3'),
                exact: true
              },
              {
                path: '/physical-ai-humanoid-robotics-book/docs/tutorial-basics/create-a-document',
                component: ComponentCreator('/physical-ai-humanoid-robotics-book/docs/tutorial-basics/create-a-document', '7a9'),
                exact: true
              },
              {
                path: '/physical-ai-humanoid-robotics-book/docs/tutorial-basics/create-a-page',
                component: ComponentCreator('/physical-ai-humanoid-robotics-book/docs/tutorial-basics/create-a-page', '1e1'),
                exact: true
              },
              {
                path: '/physical-ai-humanoid-robotics-book/docs/tutorial-basics/deploy-your-site',
                component: ComponentCreator('/physical-ai-humanoid-robotics-book/docs/tutorial-basics/deploy-your-site', '67e'),
                exact: true
              },
              {
                path: '/physical-ai-humanoid-robotics-book/docs/tutorial-basics/markdown-features',
                component: ComponentCreator('/physical-ai-humanoid-robotics-book/docs/tutorial-basics/markdown-features', 'fc2'),
                exact: true
              },
              {
                path: '/physical-ai-humanoid-robotics-book/docs/tutorial-extras/manage-docs-versions',
                component: ComponentCreator('/physical-ai-humanoid-robotics-book/docs/tutorial-extras/manage-docs-versions', '7eb'),
                exact: true
              },
              {
                path: '/physical-ai-humanoid-robotics-book/docs/tutorial-extras/translate-your-site',
                component: ComponentCreator('/physical-ai-humanoid-robotics-book/docs/tutorial-extras/translate-your-site', '5c0'),
                exact: true
              }
            ]
          }
        ]
      }
    ]
  },
  {
    path: '/physical-ai-humanoid-robotics-book/',
    component: ComponentCreator('/physical-ai-humanoid-robotics-book/', 'dfb'),
    exact: true
  },
  {
    path: '*',
    component: ComponentCreator('*'),
  },
];
