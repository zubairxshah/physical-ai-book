import { betterAuth } from "better-auth";
import { Pool } from "pg";

const pool = new Pool({
  connectionString: process.env.DATABASE_URL,
  ssl: process.env.NODE_ENV === 'production' ? { rejectUnauthorized: false } : undefined
});

export const auth = betterAuth({
  database: {
    provider: "pg",
    connection: pool
  },

  emailAndPassword: {
    enabled: true,
    requireEmailVerification: false, // For hackathon demo - enable later for production
    minPasswordLength: 8
  },

  session: {
    expiresIn: 60 * 60 * 24 * 7, // 7 days
    updateAge: 60 * 60 * 24 // Update session every 24 hours
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
    "https://physical-ai-book.vercel.app"
  ],

  advanced: {
    cookiePrefix: "physical-ai"
  }
});

export type Session = typeof auth.$Infer.Session;
export type User = typeof auth.$Infer.Session.user;
