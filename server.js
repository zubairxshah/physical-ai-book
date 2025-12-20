const express = require('express');
const cors = require('cors');
const { Pool } = require('pg');
const { betterAuth } = require('better-auth');
require('dotenv').config({ path: '.env.local' });

const app = express();

app.use(cors({
  origin: 'http://localhost:3000',
  credentials: true
}));

app.use(express.json());

// Initialize Better Auth
const pool = new Pool({
  connectionString: process.env.DATABASE_URL,
  ssl: { rejectUnauthorized: false }
});

const auth = betterAuth({
  database: {
    provider: "pg",
    connection: pool
  },
  emailAndPassword: {
    enabled: true,
    requireEmailVerification: false,
    minPasswordLength: 8
  },
  session: {
    expiresIn: 60 * 60 * 24 * 7, // 7 days
    updateAge: 60 * 60 * 24 // Update every 24 hours
  },
  user: {
    additionalFields: {
      programmingExperience: {
        type: "string",
        required: false,
        defaultValue: "beginner"
      },
      rosFamiliarity: {
        type: "string",
        required: false,
        defaultValue: "none"
      },
      aiMlBackground: {
        type: "string",
        required: false,
        defaultValue: "none"
      },
      enablePersonalization: {
        type: "boolean",
        required: false,
        defaultValue: true
      },
      enableUrduTranslation: {
        type: "boolean",
        required: false,
        defaultValue: false
      }
    }
  },
  trustedOrigins: [
    "http://localhost:3000",
    "http://localhost:8000",
    "https://physical-ai-book.vercel.app"
  ]
});

// Better Auth handler for all /api/auth/* routes
app.use('/api/auth', async (req, res) => {
  try {
    await auth.handler(req, res);
  } catch (error) {
    console.error('Auth error:', error);
    res.status(500).json({ error: 'Authentication error' });
  }
});

// Proxy to personalization API
app.use('/api/personalization', async (req, res) => {
  const targetUrl = `http://localhost:8001${req.path}`;
  try {
    const fetch = (await import('node-fetch')).default;
    const response = await fetch(targetUrl, {
      method: req.method,
      headers: { 'Content-Type': 'application/json' },
      body: req.method !== 'GET' ? JSON.stringify(req.body) : undefined
    });
    const data = await response.json();
    res.status(response.status).json(data);
  } catch (error) {
    console.error('Personalization proxy error:', error);
    res.status(500).json({ error: 'Proxy error' });
  }
});

// Proxy to translation API
app.use('/api/translation', async (req, res) => {
  const targetUrl = `http://localhost:8002${req.path}`;
  try {
    const fetch = (await import('node-fetch')).default;
    const response = await fetch(targetUrl, {
      method: req.method,
      headers: { 'Content-Type': 'application/json' },
      body: req.method !== 'GET' ? JSON.stringify(req.body) : undefined
    });
    const data = await response.json();
    res.status(response.status).json(data);
  } catch (error) {
    console.error('Translation proxy error:', error);
    res.status(500).json({ error: 'Proxy error' });
  }
});

const PORT = 8000;
app.listen(PORT, () => {
  console.log(`\n✅ Auth + API proxy server running on http://localhost:${PORT}`);
  console.log('📍 Better Auth endpoints: http://localhost:8000/api/auth/*');
  console.log('📍 Personalization API: http://localhost:8000/api/personalization/*');
  console.log('📍 Translation API: http://localhost:8000/api/translation/*');
  console.log('\n🔧 Make sure to configure Docusaurus to proxy requests to this server');
  console.log('   Add to docusaurus.config.ts:');
  console.log('   proxy: { "/api": "http://localhost:8000" }\n');
});
