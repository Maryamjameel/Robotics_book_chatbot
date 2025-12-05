import {themes as prismThemes} from 'prism-react-renderer';
import type {Config} from '@docusaurus/types';
import type * as Preset from '@docusaurus/preset-classic';
import remarkMath from 'remark-math';
import rehypeKatex from 'rehype-katex';

const config: Config = {
  title: 'Humanoid Robotics',
  tagline: 'Physical AI & Advanced Robotics Systems',
  favicon: 'img/favicon.ico',

  url: 'https://MaryamJami1.github.io',
  baseUrl: '/',
  organizationName: 'MaryamJami1',
  projectName: 'Robotics_book_chatbot',
  deploymentBranch: 'gh-pages',
  trailingSlash: false,

  onBrokenLinks: 'throw',

  markdown: {
    mermaid: true,
    hooks: {
      onBrokenMarkdownLinks: 'warn',
    },
  },

  headTags: [
    {
      tagName: 'link',
      attributes: {
        rel: 'preconnect',
        href: 'https://fonts.googleapis.com',
      },
    },
    {
      tagName: 'link',
      attributes: {
        rel: 'preconnect',
        href: 'https://fonts.gstatic.com',
        crossorigin: 'anonymous',
      },
    },
    {
      tagName: 'link',
      attributes: {
        rel: 'stylesheet',
        href: 'https://fonts.googleapis.com/css2?family=Inter:wght@300;400;500;600;700&family=JetBrains+Mono:wght@400;500;600&display=swap',
      },
    },
  ],

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
          remarkPlugins: [remarkMath],
          rehypePlugins: [[rehypeKatex, {output: 'mathml'}]],
        },
        blog: {
          showReadingTime: true,
        },
        theme: {
          customCss: './src/css/custom.css',
        },
      } satisfies Preset.Options,
    ],
  ],

  themeConfig: {
    image: 'img/docusaurus-social-card.jpg',
    colorMode: {
      defaultMode: 'dark',
      disableSwitch: false,
      respectPrefersColorScheme: false,
    },
    navbar: {
      title: 'Humanoid Robotics',
      hideOnScroll: false,
      logo: {
        alt: 'Humanoid Robotics',
        src: 'img/nao-6654027_1280.png',
        srcDark: 'img/nao-6654027_1280.png',
        width: 40,
        height: 40,
        style: {borderRadius: '8px'},
      },
      items: [
        {
          type: 'docSidebar',
          sidebarId: 'tutorialSidebar',
          position: 'left',
          label: 'Textbook',
        },
        {
          to: '/docs/glossary',
          label: 'Glossary',
          position: 'left',
        },
        {
          type: 'search',
          position: 'right',
        },
      ],
    },
    footer: {
      style: 'dark',
      links: [
        {
          title: '📚 Learning Resources',
          items: [
            {
              label: 'Introduction',
              to: '/docs/intro',
            },
            {
              label: 'Textbook Chapters',
              to: '/docs/chapters/chapter-01/lesson-01-foundations',
            },
            {
              label: 'Technical Glossary',
              to: '/docs/glossary',
            },
          ],
        },
        {
          title: '🤖 Robotics Community',
          items: [
            {
              label: 'ROS 2 Documentation',
              href: 'https://docs.ros.org/en/humble/',
            },
            {
              label: 'ROS Discourse',
              href: 'https://discourse.ros.org',
            },
            {
              label: 'Robotics Stack Exchange',
              href: 'https://robotics.stackexchange.com/',
            },
          ],
        },
        {
          title: '🔧 Development',
          items: [
            {
              label: 'GitHub Repository',
              href: 'https://github.com/MaryamJami1/Robotics_book_chatbot',
            },
            {
              label: 'Report Issues',
              href: 'https://github.com/MaryamJami1/Robotics_book_chatbot/issues',
            },
            {
              label: 'Contribute',
              href: 'https://github.com/MaryamJami1/Robotics_book_chatbot/blob/main/CONTRIBUTING.md',
            },
          ],
        },
        {
          title: '📖 About',
          items: [
            {
              label: 'About This Project',
              to: '/docs/intro',
            },
            {
              label: 'License',
              href: 'https://github.com/MaryamJami1/Robotics_book_chatbot/blob/main/LICENSE',
            },
          ],
        },
      ],
      copyright: `
        <div style="margin-top: 2rem; padding-top: 2rem; border-top: 1px solid rgba(0, 217, 255, 0.2);">
          <div style="font-size: 0.95rem; margin-bottom: 0.5rem;">
            <strong style="color: #00d9ff;">Humanoid Robotics</strong> © ${new Date().getFullYear()}
          </div>
          <div style="font-size: 0.875rem; color: #90a4ae; line-height: 1.6;">
            Empowering the next generation of robotics engineers with comprehensive knowledge and practical skills.
          </div>
          <div style="margin-top: 1rem; font-size: 0.8rem; color: #78909c;">
            Built with ❤️ using <a href="https://docusaurus.io" target="_blank" style="color: #00d9ff; text-decoration: none;">Docusaurus</a>
          </div>
        </div>
      `,
    },
    prism: {
      theme: prismThemes.vsDark,
      darkTheme: prismThemes.oneDark,
      additionalLanguages: ['python', 'cpp', 'bash', 'yaml', 'json', 'latex'],
    },
    tableOfContents: {
      minHeadingLevel: 2,
      maxHeadingLevel: 4,
    },
  } satisfies Preset.ThemeConfig,
};

export default config;
