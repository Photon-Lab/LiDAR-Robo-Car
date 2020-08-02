module.exports = {
  title: "LiDAR Robo Car",
  tagline: "A LiDAR based autonomous robocar",
  url: "https://github.com/Photon-Lab/LiDAR-Robo-Car",
  baseUrl: "/",
  favicon: "img/favicon.ico",
  organizationName: "Photon Lab - The University of Edinburgh", // Usually your GitHub org/user name.
  projectName: "LiDAR-Robo-Car", // Usually your repo name.
  themeConfig: {
    colorMode: {
      defaultMode: "dark",
      //respectPrefersColorScheme: true,
    },
    navbar: {
      title: "LiDAR Robo Car",
      logo: {
        alt: "My Site Logo",
        src: "img/logo.svg",
      },
      items: [
        {
          to: "docs/main",
          activeBasePath: "docs",
          label: "Docs",
          position: "left",
        },
        {
          href: "https://github.com/Photon-Lab/LiDAR-Robo-Car",
          position: "right",
          className: "header-github-link",
          "aria-label": "GitHub repository",
        },
      ],
    },
    colorMode: {
      defaultMode: "dark",
      disableSwitch: false,
      respectPrefersColorScheme: true,
      switchConfig: {
        darkIcon: "🌙",
        darkIconStyle: {
          // Style object passed to inline CSS
          // For more information about styling options visit: https://reactjs.org/docs/dom-elements.html#style
          marginLeft: "2px",
        },
        lightIcon: "\u2600",
        lightIconStyle: {
          marginLeft: "1px",
        },
      },
    },
    footer: {
      style: "dark",
      links: [
        {
          title: "Docs",
          items: [
            {
              label: "Style Guide",
              to: "#",
            },
            {
              label: "Second Doc",
              to: "#",
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
