/**
 * Sign in page.
 */

import React, { useState } from "react";
import Layout from "@theme/Layout";
import SignInForm from "../components/Auth/SignInForm";
import { useHistory } from "@docusaurus/router";
import styles from "../components/Auth/Auth.module.css";

export default function SignInPage() {
  const history = useHistory();
  const [error, setError] = useState<string | null>(null);

  const handleSuccess = () => {
    // Redirect to home page after successful sign in
    history.push("/");
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
