/**
 * Better Auth Server for Physical AI Book
 * Handles authentication and proxies to Python APIs
 */

const express = require('express');
const cors = require('cors');
const cookieParser = require('cookie-parser');
const { Pool } = require('pg');
const crypto = require('crypto');
const bcrypt = require('bcryptjs');
require('dotenv').config({ path: '.env.local' });

const app = express();

// Database connection
const pool = new Pool({
  connectionString: process.env.DATABASE_URL,
  ssl: { rejectUnauthorized: false },
  max: 10,
  idleTimeoutMillis: 30000,
  connectionTimeoutMillis: 10000,
});

// Middleware
app.use(cors({
  origin: ['http://localhost:3000', 'http://localhost:8000'],
  credentials: true,
  methods: ['GET', 'POST', 'PUT', 'DELETE', 'OPTIONS'],
  allowedHeaders: ['Content-Type', 'Authorization', 'Cookie'],
}));

app.use(cookieParser());
app.use(express.json());

// Helper functions
function generateSessionToken() {
  return crypto.randomBytes(32).toString('hex');
}

async function hashPassword(password) {
  return await bcrypt.hash(password, 10);
}

async function verifyPassword(password, hash) {
  return await bcrypt.compare(password, hash);
}

// ============================================================================
// AUTH ENDPOINTS
// ============================================================================

// Sign Up
app.post('/api/auth/sign-up', async (req, res) => {
  const {
    name,
    email,
    password,
    programmingExperience = 'beginner',
    rosFamiliarity = 'none',
    aiMlBackground = 'none'
  } = req.body;

  console.log('📝 Signup request for:', email);

  try {
    // Check if user exists
    const existingUser = await pool.query(
      'SELECT id FROM "user" WHERE email = $1',
      [email]
    );

    if (existingUser.rows.length > 0) {
      return res.status(400).json({ error: 'Email already registered' });
    }

    // Hash password
    const passwordHash = await hashPassword(password);

    // Create user with custom fields
    const result = await pool.query(
      `INSERT INTO "user" (
        id, name, email, "emailVerified", "createdAt", "updatedAt",
        "programmingExperience", "rosFamiliarity", "aiMlBackground",
        "enablePersonalization", "enableUrduTranslation"
      ) VALUES (
        $1, $2, $3, $4, NOW(), NOW(), $5, $6, $7, $8, $9
      ) RETURNING id, name, email`,
      [
        crypto.randomUUID(),
        name,
        email,
        false,
        programmingExperience,
        rosFamiliarity,
        aiMlBackground,
        true,
        false
      ]
    );

    // Store password hash in account table (Better Auth pattern)
    await pool.query(
      `INSERT INTO account (
        id, "userId", "accountId", provider, "accessToken", "createdAt", "updatedAt"
      ) VALUES ($1, $2, $3, $4, $5, NOW(), NOW())`,
      [
        crypto.randomUUID(),
        result.rows[0].id,
        email,
        'credential',
        passwordHash
      ]
    );

    console.log('✅ User created:', email);

    res.json({
      success: true,
      user: result.rows[0]
    });

  } catch (error) {
    console.error('❌ Signup error:', error);
    res.status(500).json({ error: 'Failed to create account' });
  }
});

// Sign In
app.post('/api/auth/sign-in', async (req, res) => {
  const { email, password } = req.body;

  console.log('🔐 Signin request for:', email);

  try {
    // Get user
    const userResult = await pool.query(
      `SELECT u.*, a."accessToken" as password_hash
       FROM "user" u
       JOIN account a ON u.id = a."userId"
       WHERE u.email = $1 AND a.provider = 'credential'`,
      [email]
    );

    if (userResult.rows.length === 0) {
      return res.status(401).json({ error: 'Invalid email or password' });
    }

    const user = userResult.rows[0];

    // Verify password
    const isValid = await verifyPassword(password, user.password_hash);
    if (!isValid) {
      return res.status(401).json({ error: 'Invalid email or password' });
    }

    // Create session
    const sessionToken = generateSessionToken();
    const expiresAt = new Date(Date.now() + 7 * 24 * 60 * 60 * 1000); // 7 days

    await pool.query(
      `INSERT INTO session (id, "userId", "expiresAt", token, "createdAt")
       VALUES ($1, $2, $3, $4, NOW())`,
      [crypto.randomUUID(), user.id, expiresAt, sessionToken]
    );

    console.log('✅ Session created for:', email);

    // Set session cookie
    res.cookie('session_token', sessionToken, {
      httpOnly: true,
      secure: false, // Set to true in production with HTTPS
      sameSite: 'lax',
      maxAge: 7 * 24 * 60 * 60 * 1000,
      path: '/'
    });

    res.json({
      success: true,
      user: {
        id: user.id,
        name: user.name,
        email: user.email,
        programmingExperience: user.programmingExperience,
        rosFamiliarity: user.rosFamiliarity,
        aiMlBackground: user.aiMlBackground,
        enablePersonalization: user.enablePersonalization,
        enableUrduTranslation: user.enableUrduTranslation
      }
    });

  } catch (error) {
    console.error('❌ Signin error:', error);
    res.status(500).json({ error: 'Failed to sign in' });
  }
});

// Get Session
app.get('/api/auth/session', async (req, res) => {
  const sessionToken = req.cookies?.session_token || req.headers.authorization?.replace('Bearer ', '');

  if (!sessionToken) {
    return res.status(401).json({ error: 'No session' });
  }

  try {
    const result = await pool.query(
      `SELECT u.* FROM "user" u
       JOIN session s ON u.id = s."userId"
       WHERE s.token = $1 AND s."expiresAt" > NOW()`,
      [sessionToken]
    );

    if (result.rows.length === 0) {
      return res.status(401).json({ error: 'Session expired' });
    }

    const user = result.rows[0];

    res.json({
      user: {
        id: user.id,
        name: user.name,
        email: user.email,
        programmingExperience: user.programmingExperience,
        rosFamiliarity: user.rosFamiliarity,
        aiMlBackground: user.aiMlBackground,
        enablePersonalization: user.enablePersonalization,
        enableUrduTranslation: user.enableUrduTranslation
      }
    });

  } catch (error) {
    console.error('❌ Session check error:', error);
    res.status(500).json({ error: 'Failed to check session' });
  }
});

// Sign Out
app.post('/api/auth/sign-out', async (req, res) => {
  const sessionToken = req.cookies?.session_token;

  if (sessionToken) {
    try {
      await pool.query('DELETE FROM session WHERE token = $1', [sessionToken]);
      console.log('✅ Session deleted');
    } catch (error) {
      console.error('❌ Signout error:', error);
    }
  }

  res.clearCookie('session_token', { path: '/' });
  res.json({ success: true });
});

// Health check
app.get('/api/auth/health', async (req, res) => {
  try {
    await pool.query('SELECT 1');
    res.json({
      status: 'healthy',
      service: 'authentication',
      database: 'connected'
    });
  } catch (error) {
    res.status(500).json({
      status: 'unhealthy',
      service: 'authentication',
      database: 'disconnected',
      error: error.message
    });
  }
});

// ============================================================================
// PROXY TO PYTHON APIS
// ============================================================================

app.use('/api/personalization', async (req, res) => {
  const targetUrl = `http://127.0.0.1:8001${req.path}`;
  try {
    const fetch = (await import('node-fetch')).default;
    const response = await fetch(targetUrl, {
      method: req.method,
      headers: { 'Content-Type': 'application/json' },
      body: req.method !== 'GET' && req.method !== 'HEAD' ? JSON.stringify(req.body) : undefined
    });
    const data = await response.json();
    res.status(response.status).json(data);
  } catch (error) {
    console.error('❌ Personalization proxy error:', error.message);
    res.status(500).json({ error: 'Proxy error' });
  }
});

app.use('/api/translation', async (req, res) => {
  const targetUrl = `http://127.0.0.1:8002${req.path}`;
  try {
    const fetch = (await import('node-fetch')).default;
    const response = await fetch(targetUrl, {
      method: req.method,
      headers: { 'Content-Type': 'application/json' },
      body: req.method !== 'GET' && req.method !== 'HEAD' ? JSON.stringify(req.body) : undefined
    });
    const data = await response.json();
    res.status(response.status).json(data);
  } catch (error) {
    console.error('❌ Translation proxy error:', error.message);
    res.status(500).json({ error: 'Proxy error' });
  }
});

// ============================================================================
// START SERVER
// ============================================================================

const PORT = 8000;

async function startServer() {
  try {
    // Test database connection
    await pool.query('SELECT 1');
    console.log('✅ Database connected');

    app.listen(PORT, () => {
      console.log('\n' + '='.repeat(70));
      console.log('🚀 Authentication Server Running');
      console.log('='.repeat(70));
      console.log(`📍 Server URL: http://localhost:${PORT}`);
      console.log(`📍 Auth endpoints: http://localhost:${PORT}/api/auth/*`);
      console.log(`📍 Personalization: http://localhost:${PORT}/api/personalization/*`);
      console.log(`📍 Translation: http://localhost:${PORT}/api/translation/*`);
      console.log('\n📋 Available endpoints:');
      console.log('   POST /api/auth/sign-up    - Create account');
      console.log('   POST /api/auth/sign-in    - Login');
      console.log('   GET  /api/auth/session    - Check session');
      console.log('   POST /api/auth/sign-out   - Logout');
      console.log('   GET  /api/auth/health     - Health check');
      console.log('='.repeat(70) + '\n');
    });

  } catch (error) {
    console.error('❌ Failed to start server:', error);
    process.exit(1);
  }
}

startServer();
