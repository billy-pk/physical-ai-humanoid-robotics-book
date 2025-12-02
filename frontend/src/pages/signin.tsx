/**
 * Sign in page.
 */

import React, { useState } from "react";
import Layout from "@theme/Layout";
import SignInForm from "../components/Auth/SignInForm";
import { useHistory } from "@docusaurus/router";
import useDocusaurusContext from "@docusaurus/useDocusaurusContext";
import styles from "../components/Auth/Auth.module.css";

export default function SignInPage() {
  const history = useHistory();
  const { siteConfig } = useDocusaurusContext();
  const [error, setError] = useState<string | null>(null);

  const handleSuccess = () => {
    // Redirect to home page after successful sign in
    // Use baseUrl to ensure correct path (e.g., /physical-ai-humanoid-robotics-book/)
    const baseUrl = siteConfig.baseUrl || "/";
    // Remove trailing slash if present, then add it back to ensure we go to home
    const homePath = baseUrl.endsWith("/") ? baseUrl : `${baseUrl}/`;
    console.log("Redirecting to home:", homePath);
    // Use window.location for a full page reload to ensure the route exists and baseUrl is handled correctly
    window.location.href = homePath;
  };

  return (
    <Layout title="Sign In" description="Sign in to your account">
      <div style={{ padding: "2rem 0", minHeight: "80vh", display: "flex", alignItems: "center", justifyContent: "center" }}>
        <div style={{ width: "100%", maxWidth: "400px" }}>
          {error && <div className={styles.error}>{error}</div>}
          <SignInForm onSuccess={handleSuccess} onError={setError} />
        </div>
      </div>
    </Layout>
  );
}
