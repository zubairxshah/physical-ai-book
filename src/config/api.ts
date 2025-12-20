// API Configuration
const IS_PRODUCTION = process.env.NODE_ENV === 'production';

// Auth API - Always goes through port 8000
export const AUTH_API_URL = IS_PRODUCTION
  ? '/api/auth'
  : 'http://localhost:8000/api/auth';

// Personalization API - Direct connection to port 8001 (or via proxy through 8000)
export const PERSONALIZATION_API_URL = IS_PRODUCTION
  ? '/api/personalization'
  : 'http://localhost:8000/api/personalization'; // Auth server proxies to 8001

// Translation API - Direct connection to port 8002 (or via proxy through 8000)
export const TRANSLATION_API_URL = IS_PRODUCTION
  ? '/api/translation'
  : 'http://localhost:8000/api/translation'; // Auth server proxies to 8002
