// API Configuration
const IS_PRODUCTION = process.env.NODE_ENV === 'production';

// Hugging Face Space URL - Update this after deploying to HF Space
// Format: https://YOUR_USERNAME-physical-ai-backend.hf.space
const HF_SPACE_URL = 'https://zubairxshah-physical-ai-backend.hf.space';

// Auth API - Always goes through port 8000 locally
export const AUTH_API_URL = IS_PRODUCTION
  ? '/api/auth'
  : 'http://localhost:8000/api/auth';

// Backend APIs - Use HF Space in production, local servers in development
export const PERSONALIZATION_API_URL = IS_PRODUCTION
  ? HF_SPACE_URL
  : 'http://localhost:8001';

export const TRANSLATION_API_URL = IS_PRODUCTION
  ? HF_SPACE_URL
  : 'http://localhost:8002';

export const CHATBOT_API_URL = IS_PRODUCTION
  ? HF_SPACE_URL
  : 'http://localhost:8003';
