//https://codesandbox.io/s/8z22h

import * as React from "react";

import VideoJSPlayer from "./vplayer";

import "./styles.css";

type props = {
  source: string;
};

export default function VPlayer({ source }: props) {
  const videoJsOptions = {
    sources: [
      {
        src: source,
        type: "video/mp4",
      },
    ],
  };

  return (
    <div className="VPlayer">
      <VideoJSPlayer options={videoJsOptions} />
    </div>
  );
}
