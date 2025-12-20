// API Configuration
const IS_PRODUCTION = process.env.NODE_ENV === 'production';

// Hugging Face Space URL - Update this after deploying to HF Space
const HF_SPACE_URL = 'https://zubairxshah-physical-ai-backend.hf.space';

// All APIs point to HF Space in production
export const AUTH_API_URL = IS_PRODUCTION
  ? HF_SPACE_URL
  : 'http://localhost:8000/api/auth';

export const PERSONALIZATION_API_URL = IS_PRODUCTION
  ? HF_SPACE_URL
  : 'http://localhost:8001';

export const TRANSLATION_API_URL = IS_PRODUCTION
  ? HF_SPACE_URL
  : 'http://localhost:8002';

export const CHATBOT_API_URL = IS_PRODUCTION
  ? HF_SPACE_URL
  : 'http://localhost:8003';
