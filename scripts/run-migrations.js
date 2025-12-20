/**
 * Database Migration Script
 * Runs Better Auth tables + custom schema
 */

require('dotenv').config({ path: '.env.local' });
const { Pool } = require('pg');
const fs = require('fs');
const path = require('path');

const pool = new Pool({
  connectionString: process.env.DATABASE_URL,
  ssl: { rejectUnauthorized: false }
});

async function runMigrations() {
  const client = await pool.connect();

  try {
    console.log('🔗 Connected to database');

    // Step 1: Create Better Auth tables
    console.log('\n📦 Creating Better Auth tables...');

    await client.query(`
      -- User table
      CREATE TABLE IF NOT EXISTS "user" (
        id TEXT PRIMARY KEY,
        email TEXT UNIQUE NOT NULL,
        "emailVerified" BOOLEAN DEFAULT FALSE,
        name TEXT,
        "createdAt" TIMESTAMP DEFAULT NOW(),
        "updatedAt" TIMESTAMP DEFAULT NOW(),

        -- Custom fields
        "programmingExperience" TEXT DEFAULT 'beginner',
        "rosFamiliarity" TEXT DEFAULT 'none',
        "aiMlBackground" TEXT DEFAULT 'none',
        "enablePersonalization" BOOLEAN DEFAULT TRUE,
        "enableUrduTranslation" BOOLEAN DEFAULT FALSE
      );
    `);
    console.log('✅ User table created');

    await client.query(`
      -- Session table
      CREATE TABLE IF NOT EXISTS session (
        id TEXT PRIMARY KEY,
        "userId" TEXT NOT NULL REFERENCES "user"(id) ON DELETE CASCADE,
        "expiresAt" TIMESTAMP NOT NULL,
        token TEXT UNIQUE NOT NULL,
        "createdAt" TIMESTAMP DEFAULT NOW()
      );
    `);
    console.log('✅ Session table created');

    await client.query(`
      -- Verification token table
      CREATE TABLE IF NOT EXISTS verification_token (
        identifier TEXT NOT NULL,
        token TEXT NOT NULL,
        expires TIMESTAMP NOT NULL,
        PRIMARY KEY (identifier, token)
      );
    `);
    console.log('✅ Verification token table created');

    await client.query(`
      -- Account table (for OAuth, if needed later)
      CREATE TABLE IF NOT EXISTS account (
        id TEXT PRIMARY KEY,
        "userId" TEXT NOT NULL REFERENCES "user"(id) ON DELETE CASCADE,
        "accountId" TEXT NOT NULL,
        provider TEXT NOT NULL,
        "accessToken" TEXT,
        "refreshToken" TEXT,
        "expiresAt" TIMESTAMP,
        "createdAt" TIMESTAMP DEFAULT NOW(),
        "updatedAt" TIMESTAMP DEFAULT NOW()
      );
    `);
    console.log('✅ Account table created');

    // Step 2: Run custom schema
    console.log('\n📦 Creating custom tables...');

    const schemaSQL = fs.readFileSync(
      path.join(__dirname, 'setup_auth_db.sql'),
      'utf8'
    );

    await client.query(schemaSQL);
    console.log('✅ Custom tables created and seeded');

    // Step 3: Verify tables
    console.log('\n🔍 Verifying tables...');

    const tables = await client.query(`
      SELECT table_name
      FROM information_schema.tables
      WHERE table_schema = 'public'
      ORDER BY table_name;
    `);

    console.log('\n📋 Tables created:');
    tables.rows.forEach(row => {
      console.log(`   - ${row.table_name}`);
    });

    // Step 4: Check seeded data
    const tooltipCount = await client.query(`
      SELECT COUNT(*) FROM personalization_tooltips;
    `);

    console.log(`\n✅ Migration complete!`);
    console.log(`   - ${tooltipCount.rows[0].count} tooltip definitions seeded`);

  } catch (error) {
    console.error('\n❌ Migration failed:', error.message);
    console.error(error.stack);
    process.exit(1);
  } finally {
    client.release();
    await pool.end();
  }
}

// Run migrations
runMigrations()
  .then(() => {
    console.log('\n🎉 Database setup complete!');
    process.exit(0);
  })
  .catch((error) => {
    console.error('\n❌ Error:', error);
    process.exit(1);
  });
