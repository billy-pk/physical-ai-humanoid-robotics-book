/**
 * Multi-step signup flow component.
 * 
 * Handles: Signup → Questionnaire → Complete
 */

import React, { useState, useEffect } from "react";
import SignUpForm from "./SignUpForm";
import UserBackgroundQuestionnaire from "./UserBackgroundQuestionnaire";
import { useAuth } from "../../contexts/AuthContext";
import { useHistory } from "@docusaurus/router";
import Link from "@docusaurus/Link";
import styles from "./SignUpFlow.module.css";

type Step = "signup" | "questionnaire" | "complete";

export default function SignUpFlow({ onComplete }: { onComplete?: () => void }) {
  const { user, isLoading } = useAuth();
  const history = useHistory();
  // Don't use user state to determine step - only move to questionnaire when signup succeeds
  const [step, setStep] = useState<Step>("signup");
  const [error, setError] = useState<string | null>(null);

  // Don't auto-navigate based on user state to avoid jumping

  const handleSignupSuccess = () => {
    console.log("handleSignupSuccess called, moving to questionnaire"); // Debug log
    setError(null);
    setStep("questionnaire");
    console.log("Step set to:", "questionnaire"); // Debug log
  };

  const handleQuestionnaireComplete = () => {
    setStep("complete");
    // Show success screen instead of immediate redirect
  };

  const handleQuestionnaireSkip = () => {
    setStep("complete");
    setTimeout(() => {
      onComplete?.();
    }, 1000);
  };

  // Only move to questionnaire when signup succeeds, not based on existing auth state
  // This prevents jumping to questionnaire if user visits signup page with stale session
  // The step will only change via handleSignupSuccess callback

  // Debug: Log step changes
  React.useEffect(() => {
    console.log("SignUpFlow step changed to:", step);
  }, [step]);

  return (
    <div className={styles.container}>
      {/* Progress Indicator */}
      <div className={styles.progress}>
        <div className={styles.progressSteps}>
          <div className={`${styles.step} ${step === "signup" ? styles.active : (step === "questionnaire" || step === "complete") ? styles.completed : ""}`}>
            <div className={styles.stepNumber}>1</div>
            <div className={styles.stepLabel}>Sign Up</div>
          </div>
          <div className={styles.progressLine}></div>
          <div className={`${styles.step} ${step === "questionnaire" ? styles.active : step === "complete" ? styles.completed : ""}`}>
            <div className={styles.stepNumber}>2</div>
            <div className={styles.stepLabel}>Background</div>
          </div>
          <div className={styles.progressLine}></div>
          <div className={`${styles.step} ${step === "complete" ? styles.active : ""}`}>
            <div className={styles.stepNumber}>3</div>
            <div className={styles.stepLabel}>Complete</div>
          </div>
        </div>
      </div>

      {/* Content */}
      <div className={styles.content}>
        {error && <div className={styles.error}>{error}</div>}

        {step === "signup" && (
          <>
            {/* Show if user is already signed in */}
            {user && (
              <div style={{ padding: "2rem", textAlign: "center" }}>
                <p style={{ marginBottom: "1rem" }}>You're already signed in as {user.email}</p>
                <div style={{ display: "flex", gap: "1rem", justifyContent: "center", flexWrap: "wrap" }}>
                  <button
                    onClick={() => setStep("questionnaire")}
                    style={{
                      padding: "0.75rem 1.5rem",
                      backgroundColor: "#28a745",
                      color: "white",
                      border: "none",
                      borderRadius: "4px",
                      cursor: "pointer",
                      fontSize: "1rem",
                    }}
                  >
                    Continue to Questionnaire
                  </button>
                  <button
                    onClick={async () => {
                      const { signOut } = await import("../../lib/auth");
                      await signOut();
                      window.location.reload();
                    }}
                    style={{
                      padding: "0.75rem 1.5rem",
                      backgroundColor: "#6c757d",
                      color: "white",
                      border: "none",
                      borderRadius: "4px",
                      cursor: "pointer",
                      fontSize: "1rem",
                    }}
                  >
                    Sign Out & Sign Up New Account
                  </button>
                </div>
              </div>
            )}

            {/* Show signup form only if not signed in */}
            {!user && !isLoading && (
              <>
                <SignUpForm onSuccess={handleSignupSuccess} onError={setError} />
                {error && error.includes("already registered") && (
                  <div style={{ marginTop: "1rem", textAlign: "center" }}>
                    <p>Already have an account?</p>
                    <button
                      onClick={() => history.push("/signin")}
                      style={{
                        marginTop: "0.5rem",
                        padding: "0.5rem 1rem",
                        backgroundColor: "#007bff",
                        color: "white",
                        border: "none",
                        borderRadius: "4px",
                        cursor: "pointer",
                      }}
                    >
                      Sign In Instead
                    </button>
                  </div>
                )}
              </>
            )}

            {/* Show loading while checking auth */}
            {isLoading && (
              <div style={{ padding: "2rem", textAlign: "center" }}>
                <p>Loading...</p>
              </div>
            )}
          </>
        )}

        {step === "questionnaire" && (
          <UserBackgroundQuestionnaire
            onComplete={handleQuestionnaireComplete}
            onCancel={handleQuestionnaireSkip}
          />
        )}

        {step === "complete" && (
          <div className={styles.complete}>
            <h2>🎉 Welcome!</h2>
            <p>Your account has been created successfully.</p>
            <p>You can now start exploring personalized content.</p>
            <Link
              to="/docs/intro"
              style={{
                display: "inline-block",
                marginTop: "2rem",
                padding: "0.75rem 2rem",
                backgroundColor: "var(--ifm-color-primary)",
                color: "white",
                border: "none",
                borderRadius: "4px",
                fontSize: "1rem",
                fontWeight: "500",
                textDecoration: "none",
                cursor: "pointer",
              }}
            >
              Start Learning →
            </Link>
          </div>
        )}
      </div>
    </div>
  );
}
