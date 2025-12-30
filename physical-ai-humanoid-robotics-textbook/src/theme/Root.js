import React from "react";
import OriginalRoot from '@theme-original/Root';
import Chatbot from "../components/Chatbot/index";
// Wrap the original Root component to include the Chatbot on every page

export default function Root({ children }) {
  return (
    <>
      <OriginalRoot>{children}</OriginalRoot>
      <Chatbot />
    </>
  );
}
