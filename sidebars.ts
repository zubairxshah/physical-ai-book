import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

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
  tutorialSidebar: [
    {
      type: 'doc',
      id: 'intro',
      label: '🏠 Home',
    },
    {
      type: 'category',
      label: '📚 Part 1: Foundations of Physical AI',
      items: [
        'chapter1',
        'chapter2',
        'chapter3',
      ],
      collapsed: false,
    },
    {
      type: 'category',
      label: '🤖 Part 2: Humanoid Robotics',
      items: [
        'chapter4',
        'chapter5',
        'chapter6',
      ],
      collapsed: false,
    },
    {
      type: 'category',
      label: '🧠 Part 3: Intelligence and Autonomy',
      items: [
        'chapter7',
        'chapter8',
        'chapter9',
      ],
      collapsed: false,
    },
    {
      type: 'category',
      label: '🚀 Part 4: Applications and Future',
      items: [
        'chapter10',
        'chapter11',
        'chapter12',
      ],
      collapsed: false,
    },
  ],
};

export default sidebars;