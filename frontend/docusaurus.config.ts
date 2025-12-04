import {themes as prismThemes} from 'prism-react-renderer';
import type {Config} from '@docusaurus/types';
import type * as Preset from '@docusaurus/preset-classic';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

const config: Config = {
  title: 'Physical AI & Humanoid Robotics Textbook',
  tagline: 'Comprehensive robotics education for Panaversity students',
  favicon: 'img/favicon.ico',

  url: 'https://MaryamJami1.github.io',
  baseUrl: '/Robotics_book_chatbot/',
  organizationName: 'MaryamJami1',
  projectName: 'Robotics_book_chatbot',
  deploymentBranch: 'gh-pages',
  trailingSlash: false,

  onBrokenLinks: 'throw',

  markdown: {
    hooks: {
      onBrokenMarkdownLinks: 'warn',
    },
  },

  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
    // Future: Add 'ur' for Urdu support
  },

  presets: [
    [
      'classic',
      {
        docs: {
          sidebarPath: './sidebars.ts',
          editUrl: 'https://github.com/MaryamJami1/Robotics_book_chatbot/tree/main/frontend/',
        },
        blog: false, // Disable blog feature for textbook
        theme: {
          customCss: './src/css/custom.css',
        },
      } satisfies Preset.Options,
    ],
  ],

  themeConfig: {
    navbar: {
      title: 'Robotics Textbook',
      logo: {
        alt: 'Panaversity Logo',
        src: 'img/logo.svg',
      },
      items: [
        {
          type: 'docSidebar',
          sidebarId: 'chapters',
          position: 'left',
          label: 'Chapters',
        },
        {
          type: 'docSidebar',
          sidebarId: 'glossary',
          position: 'left',
          label: 'Glossary',
        },
        {
          href: 'https://github.com//Robotics_book_chatbot',
          label: 'GitHub',
          position: 'right',
        },
      ],
    },
    footer: {
      style: 'dark',
      links: [],
      copyright: `Copyright © ${new Date().getFullYear()} Panaversity. Built with Docusaurus.`,
    },
    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
    },
  } satisfies Preset.ThemeConfig,
};

export default config;
