export interface PersonalizationPreferences {
  experience_level: 'beginner' | 'intermediate' | 'advanced';
  learning_topics: string[];  // 1-10 items, each 1-50 chars
  learning_goals: string;  // max 500 chars
  content_mode: 'full' | 'personalized';
  urdu_translation_enabled: boolean;
  preferences_submitted_at?: string;  // ISO 8601
  preferences_last_updated_at?: string;  // ISO 8601
  preferences_version: number;
}
