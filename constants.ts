import { RubricConfig, Student } from './types';

export const RUBRIC_PRESETS: Record<string, RubricConfig> = {
  'Standard': {
    name: 'Standard (ISO-5817-C)',
    targetWidth: 8.0,
    widthTolerance: 1.0, // 7.0 - 9.0 is acceptable
    targetHeight: 2.0,
    heightTolerance: 0.5, // 1.5 - 2.5 is acceptable
    maxPorosity: 0,
    maxSpatter: 2,
  },
  'Strict': {
    name: 'Strict (ISO-5817-B)',
    targetWidth: 8.0,
    widthTolerance: 0.5, // 7.5 - 8.5
    targetHeight: 2.0,
    heightTolerance: 0.2, // 1.8 - 2.2
    maxPorosity: 0,
    maxSpatter: 0,
  },
  'Training': {
    name: 'Novice Training',
    targetWidth: 8.0,
    widthTolerance: 2.0, // 6.0 - 10.0
    targetHeight: 2.0,
    heightTolerance: 1.0, // 1.0 - 3.0
    maxPorosity: 3, // Allows some errors
    maxSpatter: 5,
  }
};

export const DEFAULT_RUBRIC = RUBRIC_PRESETS['Standard'];

export const MOCK_STUDENTS: Student[] = [
  { id: 1, name: "Alice Chen", student_id: "S2024001", class_name: "Class 1-A", level: "Novice" },
  { id: 2, name: "Bob Smith", student_id: "S2024002", class_name: "Class 1-A", level: "Novice" },
  { id: 3, name: "Charlie Davis", student_id: "S2024003", class_name: "Class 1-B", level: "Intermediate" },
  { id: 4, name: "Diana Prince", student_id: "S2024004", class_name: "Class 2-A", level: "Advanced" },
];

// Placeholder images for the welding environment
export const CAMERA_FEED_PLACEHOLDER = "https://picsum.photos/800/600?grayscale&blur=2";
export const WELD_RESULT_PLACEHOLDER = "https://picsum.photos/800/600";