import type {Config} from '@docusaurus/types';
import type * as Preset from '@docusaurus/preset-classic';
import {existsSync, readFileSync} from 'node:fs';
import {robonixDark, robonixLight} from './src/prism-robonix';

const sourceRevision = readFileSync('ROBONIX_SOURCE_REVISION', 'utf8').trim();
const hasGitMetadata = existsSync('.git');

const config: Config = {
  title: 'Robonix 开发手册',
  tagline: '具身智能操作系统的使用、开发与本体接入文档',
  favicon: 'img/robonix-mark.svg',
  clientModules: [require.resolve('./src/prism-rosidl.ts')],
  url: 'https://robonix.syswonder.org',
  baseUrl: '/',
  organizationName: 'syswonder',
  projectName: 'robonix-book',
  onBrokenLinks: 'throw',
  onBrokenAnchors: 'throw',
  onDuplicateRoutes: 'throw',
  trailingSlash: false,
  future: {v4: true},
  i18n: {
    defaultLocale: 'zh-Hans',
    locales: ['zh-Hans'],
  },
  plugins: [],
  themes: [
    [
      '@easyops-cn/docusaurus-search-local',
      {
        hashed: true,
        language: ['en', 'zh'],
        indexDocs: true,
        indexBlog: false,
        docsRouteBasePath: '/',
      },
    ],
  ],
  presets: [
    [
      'classic',
      {
        docs: {
          routeBasePath: '/',
          sidebarPath: './sidebars.ts',
          editUrl: 'https://github.com/syswonder/robonix-book/edit/main/',
          showLastUpdateAuthor: hasGitMetadata,
          showLastUpdateTime: hasGitMetadata,
        },
        blog: false,
        theme: {
          customCss: './src/css/custom.css',
        },
      } satisfies Preset.Options,
    ],
  ],
  themeConfig: {
    announcementBar: {
      id: `source-baseline-${sourceRevision.slice(0, 8)}`,
      content:
        '<span class="source-baseline">文档基线：' +
        '<a href="https://github.com/syswonder/robonix/tree/dev-next">Robonix <strong>dev-next</strong></a>' +
        ' · <a href="https://github.com/syswonder/robonix/commit/' +
        sourceRevision +
        '"><code>' +
        sourceRevision.slice(0, 8) +
        '</code></a></span>',
      backgroundColor: '#eef1ff',
      textColor: '#283689',
      isCloseable: false,
    },
    image: 'img/robonix.svg',
    colorMode: {
      defaultMode: 'light',
      respectPrefersColorScheme: true,
    },
    navbar: {
      title: 'Robonix',
      logo: {
        alt: 'Robonix',
        src: 'img/robonix-mark.svg',
      },
      items: [
        {
          type: 'docSidebar',
          sidebarId: 'handbook',
          position: 'left',
          label: '手册',
        },
        {
          href: 'https://syswonder.github.io/robonix-package-catalog/',
          label: '软件包目录',
          position: 'right',
        },
        {
          href: 'https://github.com/syswonder/robonix',
          label: 'Robonix 源码',
          position: 'right',
        },
        {
          href: 'https://github.com/syswonder/robonix-book',
          label: '编辑手册',
          position: 'right',
        },
      ],
    },
    footer: {
      style: 'dark',
      links: [
        {
          title: '快速入口',
          items: [
            {label: 'Webots 快速上手', to: '/getting-started/quickstart'},
            {label: '开发者指南', to: '/developer-guide'},
            {label: '机器人本体接入', to: '/integration-guide/vendor-onboarding'},
          ],
        },
        {
          title: '社区',
          items: [
            {label: 'GitHub', href: 'https://github.com/syswonder/robonix'},
            {label: '软件包目录', href: 'https://syswonder.github.io/robonix-package-catalog/'},
            {label: '文档贡献', to: '/contributing/documentation'},
          ],
        },
      ],
      copyright: `Copyright © ${new Date().getFullYear()} Syswonder.`,
    },
    tableOfContents: {
      minHeadingLevel: 2,
      maxHeadingLevel: 4,
    },
    prism: {
      theme: robonixLight,
      darkTheme: robonixDark,
      additionalLanguages: ['bash', 'rust', 'toml'],
    },
  } satisfies Preset.ThemeConfig,
};

export default config;
