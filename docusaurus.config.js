// @ts-check
import {themes as prismThemes} from 'prism-react-renderer';

const config = {
  title: 'Physical AI & Humanoid Robotics',
  tagline: 'Advanced Robotics Curriculum',
  url: 'https://SHAH-MIR-USMAN.github.io',
  baseUrl: '/',
  organizationName: 'SHAH-MIR-USMAN',
  projectName: 'physical-ai-book',
  onBrokenLinks: 'ignore',
  onBrokenMarkdownLinks: 'warn',
  i18n: { defaultLocale: 'en', locales: ['en'] },

  presets: [
    [
      'classic',
      ({
        docs: {
          sidebarPath: './sidebars.js',
          showLastUpdateAuthor: false,
        },
        blog: false,
        theme: { customCss: './src/css/custom.css' },
      }),
    ],
  ],

  themeConfig: ({
    colorMode: { defaultMode: 'dark', disableSwitch: false },
    docs: {
      sidebar: {
        hideable: true,
        autoCollapseCategories: true,
      },
    },
    navbar: {
      title: 'Physical AI & Humanoid Robotics',
      logo: { alt: 'Academic Logo', src: 'https://img.icons8.com/ios-filled/100/ffffff/structural.png' },
      items: [
        { type: 'docSidebar', sidebarId: 'tutorialSidebar', position: 'left', label: 'READ TEXTBOOK' },
        { href: 'https://github.com/SHAH-MIR-USMAN/physical-ai-book', label: 'Repository', position: 'right' },
      ],
    },
    footer: {
      style: 'dark',
      copyright: '© 2025 Shah Mir Usman. GIAIC Hackathon. All Rights Reserved.',
    },
    prism: { theme: prismThemes.github, darkTheme: prismThemes.vsDark },
  }),
};
export default config;