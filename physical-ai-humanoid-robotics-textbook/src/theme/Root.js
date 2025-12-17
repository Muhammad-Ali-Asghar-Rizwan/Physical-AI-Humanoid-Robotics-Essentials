import React from "react";
import OriginalRoot from '@theme-original/Root';
import Chatbot from "../components/Chatbot/index";

export default function Root({ children }) {
  return (
    <>
      <OriginalRoot>{children}</OriginalRoot>
      <Chatbot />
    </>
  );
}
