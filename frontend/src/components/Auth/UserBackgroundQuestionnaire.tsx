import React, { useState } from "react";
import { useAuth } from "../../contexts/AuthContext";
import styles from "./Questionnaire.module.css";

interface UserBackgroundQuestionnaireProps {
  onComplete?: () => void;
  onCancel?: () => void;
}

const UserBackgroundQuestionnaire: React.FC<UserBackgroundQuestionnaireProps> = ({
  onComplete,
  onCancel,
}) => {
  const { session } = useAuth();

  const [formData, setFormData] = useState({
    software_background: [] as string[],
    hardware_background: [] as string[],
    experience_level: "",
    learning_goals: "",
    has_robotics_projects: false,
    robotics_projects_description: "",
    learning_style: undefined as string | undefined,
  });

  const [errors, setErrors] = useState<Record<string, string>>({});
  const [isSubmitting, setIsSubmitting] = useState(false);
  const [submitError, setSubmitError] = useState<string | null>(null);

  const softwareOptions = [
    "Python",
    "C++",
    "JavaScript",
    "Java",
    "ROS",
    "TensorFlow/PyTorch",
    "OpenCV",
    "None",
  ];

  const hardwareOptions = [
    "Arduino",
    "Raspberry Pi",
    "NVIDIA Jetson",
    "3D Printing",
    "Sensors/Actuators",
    "None",
  ];

  const experienceLevels = [
    { value: "beginner", label: "Beginner - New to robotics" },
    { value: "intermediate", label: "Intermediate - Some experience" },
    { value: "advanced", label: "Advanced - Professional experience" },
  ];

  const handleSoftwareChange = (option: string) => {
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

  const handleHardwareChange = (option: string) => {
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
    console.log("Validating form with data:", formData);
    const newErrors: Record<string, string> = {};

    if (formData.software_background.length === 0) {
      newErrors.software_background = "Please select at least one option or 'None'";
      console.log("Validation error: software_background is empty");
    }

    if (formData.hardware_background.length === 0) {
      newErrors.hardware_background = "Please select at least one option or 'None'";
      console.log("Validation error: hardware_background is empty");
    }

    if (!formData.experience_level) {
      newErrors.experience_level = "Please select your experience level";
      console.log("Validation error: experience_level is empty or falsy:", formData.experience_level);
    }

    if (formData.has_robotics_projects && !formData.robotics_projects_description?.trim()) {
      newErrors.robotics_projects_description = "Please describe your robotics projects";
      console.log("Validation error: robotics_projects_description is required but empty");
    }

    if (formData.learning_goals && formData.learning_goals.length > 500) {
      newErrors.learning_goals = "Learning goals must be 500 characters or less";
      console.log("Validation error: learning_goals too long");
    }

    console.log("Validation errors:", newErrors);
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
      console.log("Submitting questionnaire to FastAPI...");
      
      // Submit directly to FastAPI with credentials: 'include'
      // The browser will automatically send the Better Auth cookies
      // FastAPI will validate the session with Better Auth
      const response = await fetch("http://localhost:8000/api/auth/profile/background", {
        method: "POST",
        headers: {
          "Content-Type": "application/json",
        },
        credentials: "include", // This sends the httpOnly cookies automatically
        body: JSON.stringify({
          software_background: formData.software_background.filter((item) => item !== "None"),
          hardware_background: formData.hardware_background.filter((item) => item !== "None"),
          experience_level: formData.experience_level,
          learning_goals: formData.learning_goals || undefined,
          has_robotics_projects: formData.has_robotics_projects,
          robotics_projects_description: formData.has_robotics_projects
            ? formData.robotics_projects_description
            : undefined,
          learning_style: formData.learning_style,
        }),
      });

      console.log("FastAPI response status:", response.status);

      if (!response.ok) {
        const errorData = await response.json().catch(() => ({}));
        console.error("FastAPI error response:", errorData);
        
        // Handle validation errors (array of errors)
        if (Array.isArray(errorData.detail)) {
          const errorMessages = errorData.detail.map((err: any) => 
            `${err.loc?.join(' → ') || 'Field'}: ${err.msg}`
          ).join('\n');
          throw new Error(errorMessages || "Validation failed");
        }
        
        throw new Error(errorData.detail || "Failed to save questionnaire");
      }

      const data = await response.json();
      console.log("Success! Questionnaire saved:", data);

      // Success - complete the onboarding
      onComplete?.();
    } catch (error) {
      console.error("Error submitting questionnaire:", error);
      setSubmitError(
        error instanceof Error ? error.message : "An unexpected error occurred. Please try again."
      );
    } finally {
      setIsSubmitting(false);
    }
  };

  return (
    <div className={styles.container}>
      <div className={styles.header}>
        <h2>Welcome! Let's personalize your learning experience</h2>
        <p>Help us understand your background to provide tailored content and recommendations.</p>
      </div>

      <form onSubmit={handleSubmit} className={styles.form}>
        {/* Software Background */}
        <div className={styles.section}>
          <label className={styles.label}>
            What programming languages or frameworks do you know?
            <span className={styles.required}>*</span>
          </label>
          <div className={styles.checkboxGroup}>
            {softwareOptions.map((option) => (
              <label key={option} className={styles.checkboxLabel}>
                <input
                  type="checkbox"
                  checked={formData.software_background.includes(option)}
                  onChange={() => handleSoftwareChange(option)}
                  className={styles.checkbox}
                />
                {option}
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
            What hardware platforms have you worked with?
            <span className={styles.required}>*</span>
          </label>
          <div className={styles.checkboxGroup}>
            {hardwareOptions.map((option) => (
              <label key={option} className={styles.checkboxLabel}>
                <input
                  type="checkbox"
                  checked={formData.hardware_background.includes(option)}
                  onChange={() => handleHardwareChange(option)}
                  className={styles.checkbox}
                />
                {option}
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
            What's your overall experience level in robotics?
            <span className={styles.required}>*</span>
          </label>
          <div className={styles.radioGroup}>
            {experienceLevels.map((level) => (
              <label key={level.value} className={styles.radioLabel}>
                <input
                  type="radio"
                  name="experience_level"
                  value={level.value}
                  checked={formData.experience_level === level.value}
                  onChange={(e) => {
                    console.log("Experience level changed to:", e.target.value);
                    setFormData({ ...formData, experience_level: e.target.value });
                  }}
                  className={styles.radio}
                />
                {level.label}
              </label>
            ))}
          </div>
          {errors.experience_level && <span className={styles.fieldError}>{errors.experience_level}</span>}
        </div>

        {/* Robotics Projects */}
        <div className={styles.section}>
          <label className={styles.checkboxLabel}>
            <input
              type="checkbox"
              checked={formData.has_robotics_projects}
              onChange={(e) =>
                setFormData({ ...formData, has_robotics_projects: e.target.checked })
              }
              className={styles.checkbox}
            />
            I have worked on robotics projects
          </label>

          {formData.has_robotics_projects && (
            <div className={styles.subSection}>
              <label className={styles.label}>
                Please describe your projects
                <span className={styles.required}>*</span>
              </label>
              <textarea
                value={formData.robotics_projects_description}
                onChange={(e) =>
                  setFormData({ ...formData, robotics_projects_description: e.target.value })
                }
                className={styles.textarea}
                rows={4}
                placeholder="Tell us about the robotics projects you've worked on..."
              />
              {errors.robotics_projects_description && (
                <span className={styles.fieldError}>{errors.robotics_projects_description}</span>
              )}
            </div>
          )}
        </div>

        {/* Learning Goals */}
        <div className={styles.section}>
          <label className={styles.label}>What are your learning goals? (Optional)</label>
          <textarea
            value={formData.learning_goals}
            onChange={(e) => setFormData({ ...formData, learning_goals: e.target.value })}
            className={styles.textarea}
            rows={4}
            maxLength={500}
            placeholder="e.g., Build an autonomous robot, understand computer vision for robotics, learn ROS 2..."
          />
          <span className={styles.charCount}>{formData.learning_goals.length}/500</span>
          {errors.learning_goals && <span className={styles.fieldError}>{errors.learning_goals}</span>}
        </div>

        {/* Learning Style (Optional) */}
        <div className={styles.section}>
          <label className={styles.label}>Preferred learning style (Optional)</label>
          <select
            value={formData.learning_style || ""}
            onChange={(e) => {
              console.log("Learning style changed to:", e.target.value);
              setFormData({
                ...formData,
                learning_style: e.target.value || undefined,
              });
            }}
            className={styles.select}
          >
            <option value="">Select a preference...</option>
            <option value="hands-on">Hands-on / Learning by doing</option>
            <option value="theory-first">Theory first, then practice</option>
            <option value="project-based">Project-based learning</option>
            <option value="mixed">Mixed approach</option>
          </select>
        </div>

        {submitError && <div className={styles.submitError}>{submitError}</div>}

        <div className={styles.buttonGroup}>
          {onCancel && (
            <button
              type="button"
              onClick={onCancel}
              disabled={isSubmitting}
              className={styles.skipButton}
            >
              Skip for Now
            </button>
          )}
          <button type="submit" disabled={isSubmitting} className={styles.submitButton}>
            {isSubmitting ? "Saving..." : "Complete Setup"}
          </button>
        </div>
      </form>
    </div>
  );
};

export default UserBackgroundQuestionnaire;
