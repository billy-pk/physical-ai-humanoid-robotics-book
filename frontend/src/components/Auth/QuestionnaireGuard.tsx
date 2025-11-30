/**
 * Questionnaire Guard component.
 * 
 * Checks if user has completed questionnaire and redirects if not.
 * Can be used to wrap pages that require questionnaire completion.
 */

import React, { useEffect, useState } from "react";
import { useAuth } from "../../contexts/AuthContext";
import { useHistory } from "@docusaurus/router";
import UserBackgroundQuestionnaire from "./UserBackgroundQuestionnaire";

interface QuestionnaireGuardProps {
  children: React.ReactNode;
  redirectTo?: string;
  showQuestionnaire?: boolean;
}

export default function QuestionnaireGuard({
  children,
  redirectTo = "/signup",
  showQuestionnaire = true,
}: QuestionnaireGuardProps) {
  const { user, isLoading } = useAuth();
  const history = useHistory();
  const [questionnaireCompleted, setQuestionnaireCompleted] = useState<boolean | null>(null);
  const [checking, setChecking] = useState(true);

  useEffect(() => {
    const checkQuestionnaireStatus = async () => {
      if (!user) {
        setChecking(false);
        return;
      }

      try {
        // Get session token from Better Auth API
        // Better Auth cookies are set for localhost:3001, so we can't read them from localhost:3000
        const sessionResponse = await fetch("http://localhost:3001/api/auth/get-session", {
          credentials: "include", // Include cookies for Better Auth domain
        });

        if (!sessionResponse.ok) {
          setChecking(false);
          return;
        }

        const sessionData = await sessionResponse.json();
        const sessionToken = sessionData?.session?.token || sessionData?.session?.id;

        if (!sessionToken) {
          setChecking(false);
          return;
        }

        // Check profile status
        // Send token in Authorization header since cookies are domain-specific
        const response = await fetch("http://localhost:8000/api/auth/profile", {
          headers: {
            "Authorization": `Bearer ${sessionToken}`,
          },
          credentials: "include",
        });

        if (response.ok) {
          const profile = await response.json();
          setQuestionnaireCompleted(profile.questionnaire_completed || false);
        } else {
          setQuestionnaireCompleted(false);
        }
      } catch (error) {
        console.error("Error checking questionnaire status:", error);
        setQuestionnaireCompleted(false);
      } finally {
        setChecking(false);
      }
    };

    checkQuestionnaireStatus();
  }, [user]);

  if (isLoading || checking) {
    return (
      <div style={{ padding: "2rem", textAlign: "center" }}>
        <p>Loading...</p>
      </div>
    );
  }

  if (!user) {
    return <>{children}</>;
  }

  if (questionnaireCompleted === false && showQuestionnaire) {
    return (
      <div style={{ padding: "2rem" }}>
        <UserBackgroundQuestionnaire
          onComplete={() => {
            setQuestionnaireCompleted(true);
            // Optionally redirect after completion
            if (redirectTo) {
              setTimeout(() => history.push(redirectTo), 1000);
            }
          }}
        />
      </div>
    );
  }

  return <>{children}</>;
}
