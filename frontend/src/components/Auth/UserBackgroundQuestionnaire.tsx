/**
 * User Background Questionnaire component.
 * 
 * Collects user's software/hardware background, experience level, and learning goals
 * to personalize content delivery.
 */

import React, { useState } from "react";
import { useAuth } from "../../contexts/AuthContext";
import styles from "./Questionnaire.module.css";

interface QuestionnaireData {
  software_background: string[];
  hardware_background: string[];
  experience_level: "beginner" | "intermediate" | "advanced";
  learning_goals: string;
  has_robotics_projects: boolean;
  robotics_projects_description?: string;
  programming_years?: number;
  learning_style?: "visual" | "hands-on" | "theoretical" | "mixed";
}

interface UserBackgroundQuestionnaireProps {
  onComplete?: () => void;
  onCancel?: () => void;
}

const SOFTWARE_OPTIONS = [
  "Python",
  "JavaScript/TypeScript",
  "C++",
  "C",
  "Java",
  "ROS (Robot Operating System)",
  "MATLAB",
  "Rust",
  "Go",
  "None",
];

const HARDWARE_OPTIONS = [
  "Arduino",
  "Raspberry Pi",
  "ESP32/ESP8266",
  "ROS-compatible robots",
  "Sensors (camera, LiDAR, IMU, etc.)",
  "Microcontrollers",
  "3D Printing",
  "CNC Machining",
  "None",
];

export default function UserBackgroundQuestionnaire({
  onComplete,
  onCancel,
}: UserBackgroundQuestionnaireProps) {
  const { user } = useAuth();
  const [formData, setFormData] = useState<QuestionnaireData>({
    software_background: [],
    hardware_background: [],
    experience_level: "beginner",
    learning_goals: "",
    has_robotics_projects: false,
    robotics_projects_description: "",
    programming_years: undefined,
    learning_style: undefined,
  });

  const [errors, setErrors] = useState<Record<string, string>>({});
  const [isSubmitting, setIsSubmitting] = useState(false);
  const [submitError, setSubmitError] = useState<string | null>(null);

  const handleSoftwareToggle = (option: string) => {
    setFormData((prev) => {
      const current = prev.software_background;
      if (option === "None") {
        return { ...prev, software_background: ["None"] };
      }
      const filtered = current.filter((item) => item !== "None");
      if (filtered.includes(option)) {
        return { ...prev, software_background: filtered.filter((item) => item !== option) };
      }
      return { ...prev, software_background: [...filtered, option] };
    });
  };

  const handleHardwareToggle = (option: string) => {
    setFormData((prev) => {
      const current = prev.hardware_background;
      if (option === "None") {
        return { ...prev, hardware_background: ["None"] };
      }
      const filtered = current.filter((item) => item !== "None");
      if (filtered.includes(option)) {
        return { ...prev, hardware_background: filtered.filter((item) => item !== option) };
      }
      return { ...prev, hardware_background: [...filtered, option] };
    });
  };

  const validateForm = (): boolean => {
    const newErrors: Record<string, string> = {};

    if (formData.software_background.length === 0) {
      newErrors.software_background = "Please select at least one option or 'None'";
    }

    if (formData.hardware_background.length === 0) {
      newErrors.hardware_background = "Please select at least one option or 'None'";
    }

    if (!formData.experience_level) {
      newErrors.experience_level = "Please select your experience level";
    }

    if (formData.has_robotics_projects && !formData.robotics_projects_description?.trim()) {
      newErrors.robotics_projects_description = "Please describe your robotics projects";
    }

    if (formData.learning_goals && formData.learning_goals.length > 500) {
      newErrors.learning_goals = "Learning goals must be 500 characters or less";
    }

    setErrors(newErrors);
    return Object.keys(newErrors).length === 0;
  };

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setSubmitError(null);

    if (!validateForm()) {
      return;
    }

    setIsSubmitting(true);

    try {
      // Get session token from Better Auth API
      // Better Auth cookies are set for localhost:3001, so we can't read them from localhost:3000
      // Instead, we fetch the session from Better Auth which will include the token in the response
      console.log("Fetching session from Better Auth..."); // Debug
      const sessionResponse = await fetch("http://localhost:3001/api/auth/get-session", {
        credentials: "include", // Include cookies for Better Auth domain
      });

      console.log("Session response status:", sessionResponse.status); // Debug

      if (!sessionResponse.ok) {
        console.error("Failed to get session:", sessionResponse.status);
        throw new Error("Not authenticated. Please sign in again.");
      }

      const sessionData = await sessionResponse.json();
      console.log("Session data:", sessionData); // Debug
      
      // Better Auth returns session token in session.token
      let sessionToken = sessionData?.session?.token || sessionData?.session?.id;

      console.log("Session token:", sessionToken ? `${sessionToken.substring(0, 20)}...` : "MISSING"); // Debug

      if (!sessionToken) {
        console.error("No session token found in response");
        throw new Error("Not authenticated. Please sign in again.");
      }

      // Submit to backend API
      // Send session token in Authorization header since cookies are domain-specific
      console.log("Sending questionnaire data to FastAPI..."); // Debug
      const response = await fetch("http://localhost:8000/api/auth/profile/background", {
        method: "POST",
        headers: {
          "Content-Type": "application/json",
          "Authorization": `Bearer ${sessionToken}`, // Send token in Authorization header
        },
        credentials: "include",
        body: JSON.stringify({
          software_background: formData.software_background.filter((item) => item !== "None"),
          hardware_background: formData.hardware_background.filter((item) => item !== "None"),
          experience_level: formData.experience_level,
          learning_goals: formData.learning_goals || undefined,
          has_robotics_projects: formData.has_robotics_projects,
          robotics_projects_description: formData.has_robotics_projects
            ? formData.robotics_projects_description
            : undefined,
          programming_years: formData.programming_years,
          learning_style: formData.learning_style,
        }),
      });

      console.log("FastAPI response status:", response.status); // Debug

      if (!response.ok) {
        const errorData = await response.json().catch(() => ({}));
        console.error("FastAPI error:", errorData); // Debug
        throw new Error(errorData.detail || "Failed to save questionnaire");
      }

      const responseData = await response.json();
      console.log("Success! Response:", responseData); // Debug

      // Success
      onComplete?.();
    } catch (error: any) {
      console.error("Questionnaire submission error:", error); // Debug
      setSubmitError(error.message || "An error occurred while saving your information");
    } finally {
      setIsSubmitting(false);
    }
  };

  return (
    <div className={styles.questionnaire}>
      <div className={styles.header}>
        <h2>Tell Us About Yourself</h2>
        <p>Help us personalize your learning experience by sharing your background.</p>
      </div>

      <form onSubmit={handleSubmit} className={styles.form}>
        {submitError && <div className={styles.error}>{submitError}</div>}

        {/* Software Background */}
        <div className={styles.section}>
          <label className={styles.label}>
            Software Background <span className={styles.required}>*</span>
          </label>
          <p className={styles.helpText}>Select all that apply</p>
          <div className={styles.checkboxGroup}>
            {SOFTWARE_OPTIONS.map((option) => (
              <label key={option} className={styles.checkbox}>
                <input
                  type="checkbox"
                  checked={formData.software_background.includes(option)}
                  onChange={() => handleSoftwareToggle(option)}
                />
                <span>{option}</span>
              </label>
            ))}
          </div>
          {errors.software_background && (
            <span className={styles.fieldError}>{errors.software_background}</span>
          )}
        </div>

        {/* Hardware Background */}
        <div className={styles.section}>
          <label className={styles.label}>
            Hardware Background <span className={styles.required}>*</span>
          </label>
          <p className={styles.helpText}>Select all that apply</p>
          <div className={styles.checkboxGroup}>
            {HARDWARE_OPTIONS.map((option) => (
              <label key={option} className={styles.checkbox}>
                <input
                  type="checkbox"
                  checked={formData.hardware_background.includes(option)}
                  onChange={() => handleHardwareToggle(option)}
                />
                <span>{option}</span>
              </label>
            ))}
          </div>
          {errors.hardware_background && (
            <span className={styles.fieldError}>{errors.hardware_background}</span>
          )}
        </div>

        {/* Experience Level */}
        <div className={styles.section}>
          <label className={styles.label}>
            Experience Level <span className={styles.required}>*</span>
          </label>
          <div className={styles.radioGroup}>
            {(["beginner", "intermediate", "advanced"] as const).map((level) => (
              <label key={level} className={styles.radio}>
                <input
                  type="radio"
                  name="experience_level"
                  value={level}
                  checked={formData.experience_level === level}
                  onChange={(e) =>
                    setFormData((prev) => ({
                      ...prev,
                      experience_level: e.target.value as "beginner" | "intermediate" | "advanced",
                    }))
                  }
                />
                <span>{level.charAt(0).toUpperCase() + level.slice(1)}</span>
              </label>
            ))}
          </div>
          {errors.experience_level && (
            <span className={styles.fieldError}>{errors.experience_level}</span>
          )}
        </div>

        {/* Programming Years */}
        <div className={styles.section}>
          <label className={styles.label}>Years of Programming Experience</label>
          <input
            type="number"
            min="0"
            max="50"
            value={formData.programming_years || ""}
            onChange={(e) =>
              setFormData((prev) => ({
                ...prev,
                programming_years: e.target.value ? parseInt(e.target.value, 10) : undefined,
              }))
            }
            className={styles.input}
            placeholder="e.g., 5"
          />
        </div>

        {/* Learning Goals */}
        <div className={styles.section}>
          <label className={styles.label}>Learning Goals</label>
          <p className={styles.helpText}>
            What do you hope to achieve? (Optional, max 500 characters)
          </p>
          <textarea
            value={formData.learning_goals}
            onChange={(e) => setFormData((prev) => ({ ...prev, learning_goals: e.target.value }))}
            className={styles.textarea}
            rows={4}
            maxLength={500}
            placeholder="e.g., Build a humanoid robot that can navigate autonomously..."
          />
          <div className={styles.charCount}>
            {formData.learning_goals.length}/500 characters
          </div>
          {errors.learning_goals && (
            <span className={styles.fieldError}>{errors.learning_goals}</span>
          )}
        </div>

        {/* Has Robotics Projects */}
        <div className={styles.section}>
          <label className={styles.checkbox}>
            <input
              type="checkbox"
              checked={formData.has_robotics_projects}
              onChange={(e) =>
                setFormData((prev) => ({
                  ...prev,
                  has_robotics_projects: e.target.checked,
                  robotics_projects_description: e.target.checked ? prev.robotics_projects_description : "",
                }))
              }
            />
            <span>I have prior robotics projects</span>
          </label>
        </div>

        {/* Robotics Projects Description */}
        {formData.has_robotics_projects && (
          <div className={styles.section}>
            <label className={styles.label}>
              Describe Your Robotics Projects <span className={styles.required}>*</span>
            </label>
            <textarea
              value={formData.robotics_projects_description || ""}
              onChange={(e) =>
                setFormData((prev) => ({
                  ...prev,
                  robotics_projects_description: e.target.value,
                }))
              }
              className={styles.textarea}
              rows={3}
              placeholder="Tell us about your robotics projects..."
              required
            />
            {errors.robotics_projects_description && (
              <span className={styles.fieldError}>{errors.robotics_projects_description}</span>
            )}
          </div>
        )}

        {/* Learning Style */}
        <div className={styles.section}>
          <label className={styles.label}>Preferred Learning Style</label>
          <select
            value={formData.learning_style || ""}
            onChange={(e) =>
              setFormData((prev) => ({
                ...prev,
                learning_style: e.target.value
                  ? (e.target.value as "visual" | "hands-on" | "theoretical" | "mixed")
                  : undefined,
              }))
            }
            className={styles.select}
          >
            <option value="">Select an option...</option>
            <option value="visual">Visual (diagrams, videos, images)</option>
            <option value="hands-on">Hands-on (practical exercises, building)</option>
            <option value="theoretical">Theoretical (concepts, principles, theory)</option>
            <option value="mixed">Mixed (combination of approaches)</option>
          </select>
        </div>

        {/* Actions */}
        <div className={styles.actions}>
          {onCancel && (
            <button type="button" onClick={onCancel} className={styles.buttonSecondary} disabled={isSubmitting}>
              Skip for Now
            </button>
          )}
          <button type="submit" className={styles.buttonPrimary} disabled={isSubmitting}>
            {isSubmitting ? "Saving..." : "Save & Continue"}
          </button>
        </div>
      </form>
    </div>
  );
}
