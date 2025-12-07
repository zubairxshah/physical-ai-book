import {themes as prismThemes} from 'prism-react-renderer';
import type {Config} from '@docusaurus/types';
import type * as Preset from '@docusaurus/preset-classic';

const config: Config = {
  title: 'Physical AI and Humanoid Robotics',
  tagline: 'The Future of Intelligent Machines',
  favicon: 'img/favicon.ico',

  // Set the production url of your site here
  url: 'https://physical-ai-book.vercel.app',
  // Set the /<baseUrl>/ pathname under which your site is served
  baseUrl: '/',

  // GitHub pages deployment config.
  organizationName: 'zubairxshah', // Usually your GitHub org/user name.
  projectName: 'physical-ai-book', // Usually your repo name.
  deploymentBranch: 'gh-pages',
  trailingSlash: false,

  onBrokenLinks: 'throw',
  onBrokenMarkdownLinks: 'warn',

  // Even if you don't use internationalization, you can use this field to set
  // useful metadata like html lang. For example, if your site is Chinese, you
  // may want to replace "en" with "zh-Hans".
  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  presets: [
    [
      'classic',
      {
        docs: {
          sidebarPath: './sidebars.ts',
          routeBasePath: '/', // Makes docs the home page
          // Please change this to your repo.
          // Remove this to remove the "edit this page" links.
          editUrl:
            'https://github.com/zubairxshah/physical-ai-book/tree/main/',
        },
        blog: false, // Disable blog
        theme: {
          customCss: './src/css/custom.css',
        },
      } satisfies Preset.Options,
    ],
  ],

  themeConfig: {
    // Replace with your project's social card
    image: 'img/docusaurus-social-card.jpg',
    navbar: {
      title: 'Physical AI & Humanoid Robotics',
      logo: {
        alt: 'Physical AI Logo',
        src: 'img/logo.svg',
      },
      items: [
        {
          to: '/',
          position: 'left',
          label: '📚 Book',
        },
        {
          to: '/chapter1',
          position: 'left',
          label: 'Start Reading',
        },
        {
          href: 'https://github.com/zubairxshah/physical-ai-book',
          label: 'GitHub',
          position: 'right',
        },
      ],
    },
    footer: {
      style: 'dark',
      links: [
        {
          title: 'Book',
          items: [
            {
              label: 'Introduction',
              to: '/',
            },
            {
              label: 'Chapter 1: Introduction to Physical AI',
              to: '/chapter1',
            },
          ],
        },
        {
          title: 'Resources',
          items: [
            {
              label: 'GitHub Repository',
              href: 'https://github.com/zubairxshah/physical-ai-book',
            },
            {
              label: 'GIAIC',
              href: 'https://www.giaic.com/',
            },
          ],
        },
        {
          title: 'More',
          items: [
            {
              label: 'About Physical AI',
              to: '/chapter1',
            },
            {
              label: 'Future of Robotics',
              to: '/chapter12',
            },
          ],
        },
      ],
      copyright: `Copyright © ${new Date().getFullYear()} Physical AI Book. Built with Docusaurus for GIAIC Hackathon.`,
    },
    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
      additionalLanguages: ['python', 'bash', 'json', 'typescript'],
    },
  } satisfies Preset.ThemeConfig,

  // Chatbot disabled - deploy backend first, then uncomment and update API_URL
  scripts: [
    {
      src: '/chatbot-widget-v2.js',
      async: true,
    },
  ],
};

export default config;