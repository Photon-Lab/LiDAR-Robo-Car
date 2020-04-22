module.exports = {
  title: "LiDAR Robo Car",
  tagline: "A LiDAR based autonomous robocar",
  url: "https://your-docusaurus-test-site.com",
  baseUrl: "/",
  favicon: "img/favicon.ico",
  organizationName: "Photon Lab - The University of Edinburgh", // Usually your GitHub org/user name.
  projectName: "LiDAR-Robo-Car", // Usually your repo name.
  themeConfig: {
    navbar: {
      title: "LiDAR Robo Car",
      logo: {
        alt: "My Site Logo",
        src: "img/logo.svg",
      },
      links: [
        {
          to: "docs/main",
          activeBasePath: "docs",
          label: "Docs",
          position: "left",
        },
        {
          href: "https://github.com/Photon-Lab/LiDAR-Robo-Car",
          label: "GitHub",
          position: "right",
        },
      ],
    },
    footer: {
      style: "dark",
      links: [
        {
          title: "Docs",
          items: [
            {
              label: "Style Guide",
              to: "docs/doc1",
            },
            {
              label: "Second Doc",
              to: "docs/doc2",
            },
          ],
        },
        {
          title: "Community",
          items: [
            {
              label: "Stack Overflow",
              href: "https://stackoverflow.com/questions/tagged/docusaurus",
            },
            {
              label: "Discord",
              href: "https://discordapp.com/invite/docusaurus",
            },
          ],
        },
        {
          title: "Social",
          items: [
            {
              label: "Blog",
              to: "blog",
            },
            {
              label: "GitHub",
              href: "https://github.com/facebook/docusaurus",
            },
            {
              label: "Twitter",
              href: "https://twitter.com/docusaurus",
            },
          ],
        },
      ],
      copyright: `Copyright © ${new Date().getFullYear()} PhotonLab. Built with Docusaurus.`,
    },
  },
  presets: [
    [
      "@docusaurus/preset-classic",
      {
        docs: {
          sidebarPath: require.resolve("./sidebars.js"),
          editUrl: "https://github.com/facebook/docusaurus/edit/master/website/",
        },
        theme: {
          customCss: require.resolve("./src/css/custom.css"),
        },
      },
    ],
  ],
};
