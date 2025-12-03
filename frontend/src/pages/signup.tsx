/**
 * Signup page with multi-step flow.
 */

import React from "react";
import Layout from "@theme/Layout";
import SignUpFlow from "../components/Auth/SignUpFlow";
import { useHistory } from "@docusaurus/router";
import useDocusaurusContext from "@docusaurus/useDocusaurusContext";

export default function SignupPage() {
  const history = useHistory();
  const { siteConfig } = useDocusaurusContext();
  const baseUrl = siteConfig.baseUrl || '/';

  const handleComplete = () => {
    // Redirect to docs intro page after signup completion
    console.log("Signup complete, redirecting to docs...");
    // Use window.location with proper baseUrl for a full page reload to ensure the route exists
    window.location.href = `${baseUrl}docs/intro`;
  };

  return (
    <Layout title="Sign Up" description="Create your account to get started">
      <div style={{ padding: "2rem 0", minHeight: "80vh" }}>
        <SignUpFlow onComplete={handleComplete} />
      </div>
    </Layout>
  );
}
