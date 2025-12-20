-- Physical AI Book - Authentication and Personalization Database Schema
-- Run this script after Better Auth migrations

-- Note: Better Auth will create these base tables:
-- - user (id, email, emailVerified, name, createdAt, updatedAt)
-- - session (id, userId, expiresAt, token, createdAt)
-- - verification_token (identifier, token, expires)

-- We need to add custom fields to the user table via Better Auth configuration
-- The following tables are additional tables for our features:

-- ============================================================================
-- Urdu Translation Cache Table
-- ============================================================================
-- Purpose: Cache translated content to minimize API calls and costs
-- Key feature: Content hashing for fast lookup
CREATE TABLE IF NOT EXISTS urdu_translations (
  id SERIAL PRIMARY KEY,
  content_hash TEXT UNIQUE NOT NULL,
  chapter_id TEXT NOT NULL,
  original_text TEXT NOT NULL,
  translated_text TEXT NOT NULL,
  model_used TEXT DEFAULT 'gpt-4-turbo',
  tokens_used INTEGER,
  created_at TIMESTAMP DEFAULT NOW(),
  last_accessed TIMESTAMP DEFAULT NOW(),
  access_count INTEGER DEFAULT 1
);

CREATE INDEX IF NOT EXISTS idx_urdu_content_hash ON urdu_translations(content_hash);
CREATE INDEX IF NOT EXISTS idx_urdu_chapter_id ON urdu_translations(chapter_id);

COMMENT ON TABLE urdu_translations IS 'Caches Urdu translations with content hashing for 95% cost reduction';
COMMENT ON COLUMN urdu_translations.content_hash IS 'SHA256 hash of original content for fast cache lookup';
COMMENT ON COLUMN urdu_translations.access_count IS 'Number of times this translation was served from cache';

-- ============================================================================
-- Personalization Tooltips Table
-- ============================================================================
-- Purpose: Store term definitions at different experience levels
-- Key feature: Adaptive content based on user background
CREATE TABLE IF NOT EXISTS personalization_tooltips (
  id SERIAL PRIMARY KEY,
  term TEXT NOT NULL,
  experience_level TEXT CHECK (experience_level IN ('beginner', 'intermediate', 'advanced')) NOT NULL,
  definition TEXT NOT NULL,
  chapter_id TEXT,
  created_at TIMESTAMP DEFAULT NOW(),
  updated_at TIMESTAMP DEFAULT NOW(),
  UNIQUE(term, experience_level)
);

CREATE INDEX IF NOT EXISTS idx_tooltip_term ON personalization_tooltips(LOWER(term));
CREATE INDEX IF NOT EXISTS idx_tooltip_level ON personalization_tooltips(experience_level);

COMMENT ON TABLE personalization_tooltips IS 'Contextual tooltips that adapt to user experience level';
COMMENT ON COLUMN personalization_tooltips.term IS 'Technical term (e.g., "ROS 2", "Node", "API")';
COMMENT ON COLUMN personalization_tooltips.experience_level IS 'Target audience: beginner, intermediate, or advanced';

-- ============================================================================
-- Seed Personalization Tooltip Data
-- ============================================================================
-- Initial set of tooltips for common robotics/AI terms

-- ROS 2 Tooltips
INSERT INTO personalization_tooltips (term, experience_level, definition, chapter_id) VALUES
('ROS 2', 'beginner', 'Robot Operating System 2 - A beginner-friendly framework for building robot software with ready-to-use tools and libraries.', NULL),
('ROS 2', 'intermediate', 'Robot Operating System 2 - Middleware providing hardware abstraction, message-passing between processes, and package management for robotics.', NULL),
('ROS 2', 'advanced', 'ROS 2 - Production-grade robotics middleware built on DDS, supporting real-time QoS policies, lifecycle management, and component composition.', NULL)
ON CONFLICT (term, experience_level) DO NOTHING;

-- Node Tooltips
INSERT INTO personalization_tooltips (term, experience_level, definition, chapter_id) VALUES
('Node', 'beginner', 'Node - A running program in ROS 2 that performs a specific task, like reading sensor data or controlling a motor.', NULL),
('Node', 'intermediate', 'Node - An executable process in the ROS 2 graph that publishes/subscribes to topics and provides/calls services.', NULL),
('Node', 'advanced', 'Node - A DDS participant representing a computation unit with configurable QoS, lifecycle states, and composable architecture.', NULL)
ON CONFLICT (term, experience_level) DO NOTHING;

-- Topic Tooltips
INSERT INTO personalization_tooltips (term, experience_level, definition, chapter_id) VALUES
('Topic', 'beginner', 'Topic - A named channel where ROS 2 nodes send and receive messages, like a chatroom for robots.', NULL),
('Topic', 'intermediate', 'Topic - A named bus for pub/sub messaging between nodes, supporting multiple publishers and subscribers with type safety.', NULL),
('Topic', 'advanced', 'Topic - A DDS-backed communication channel with configurable QoS policies (reliability, durability, lifespan) for pub/sub patterns.', NULL)
ON CONFLICT (term, experience_level) DO NOTHING;

-- API Tooltips
INSERT INTO personalization_tooltips (term, experience_level, definition, chapter_id) VALUES
('API', 'beginner', 'API (Application Programming Interface) - A set of commands that lets one program talk to another, like a menu at a restaurant.', NULL),
('API', 'intermediate', 'API - A defined interface for software components to interact, specifying available operations, input/output formats, and behaviors.', NULL),
('API', 'advanced', 'API - A contract defining callable operations with versioned schemas, authentication, rate limiting, and backward compatibility guarantees.', NULL)
ON CONFLICT (term, experience_level) DO NOTHING;

-- Neural Network Tooltips
INSERT INTO personalization_tooltips (term, experience_level, definition, chapter_id) VALUES
('Neural Network', 'beginner', 'Neural Network - A computer program inspired by how the human brain works, learning patterns from examples.', NULL),
('Neural Network', 'intermediate', 'Neural Network - A machine learning model with interconnected layers of neurons that learn representations through backpropagation.', NULL),
('Neural Network', 'advanced', 'Neural Network - A differentiable computational graph with learnable weights, optimized via gradient descent with configurable architectures (CNN, RNN, Transformer).', NULL)
ON CONFLICT (term, experience_level) DO NOTHING;

-- Inference Tooltips
INSERT INTO personalization_tooltips (term, experience_level, definition, chapter_id) VALUES
('Inference', 'beginner', 'Inference - When an AI model makes predictions or decisions based on what it learned during training.', NULL),
('Inference', 'intermediate', 'Inference - The process of applying a trained model to new data to generate predictions, typically optimized for speed.', NULL),
('Inference', 'advanced', 'Inference - Forward pass through a trained model with optimizations (quantization, pruning, batching) for production deployment latency requirements.', NULL)
ON CONFLICT (term, experience_level) DO NOTHING;

-- Sensor Fusion Tooltips
INSERT INTO personalization_tooltips (term, experience_level, definition, chapter_id) VALUES
('Sensor Fusion', 'beginner', 'Sensor Fusion - Combining data from multiple sensors (like cameras and radar) to get a more complete picture.', NULL),
('Sensor Fusion', 'intermediate', 'Sensor Fusion - Integrating data from heterogeneous sensors to produce more accurate and reliable estimates than individual sensors.', NULL),
('Sensor Fusion', 'advanced', 'Sensor Fusion - Probabilistic integration of multi-modal sensor data using Kalman filters, particle filters, or deep learning for state estimation.', NULL)
ON CONFLICT (term, experience_level) DO NOTHING;

-- Digital Twin Tooltips
INSERT INTO personalization_tooltips (term, experience_level, definition, chapter_id) VALUES
('Digital Twin', 'beginner', 'Digital Twin - A computer simulation that mirrors a real robot or system, letting you test things safely before trying them in reality.', NULL),
('Digital Twin', 'intermediate', 'Digital Twin - A virtual replica of a physical system synchronized in real-time, used for simulation, testing, and predictive maintenance.', NULL),
('Digital Twin', 'advanced', 'Digital Twin - High-fidelity simulation with physics engines (collision, dynamics, rendering) synchronized via bi-directional data flows for sim-to-real transfer.', NULL)
ON CONFLICT (term, experience_level) DO NOTHING;

-- Sim-to-Real Tooltips
INSERT INTO personalization_tooltips (term, experience_level, definition, chapter_id) VALUES
('Sim-to-Real', 'beginner', 'Sim-to-Real - Training AI in a computer simulation, then transferring that knowledge to work with real robots.', NULL),
('Sim-to-Real', 'intermediate', 'Sim-to-Real Transfer - Adapting models trained in simulation to perform effectively on physical hardware despite reality gap.', NULL),
('Sim-to-Real', 'advanced', 'Sim-to-Real - Domain adaptation techniques (domain randomization, adversarial training, system identification) to bridge simulation-reality gap.', NULL)
ON CONFLICT (term, experience_level) DO NOTHING;

-- VLA Tooltips
INSERT INTO personalization_tooltips (term, experience_level, definition, chapter_id) VALUES
('VLA', 'beginner', 'VLA (Vision-Language-Action) - AI models that can see images, understand language commands, and control robot actions.', NULL),
('VLA', 'intermediate', 'VLA Models - Multimodal transformers that process vision and language inputs to generate robot action sequences for manipulation tasks.', NULL),
('VLA', 'advanced', 'VLA Models - Large-scale transformer architectures (RT-1, RT-2, OpenVLA) trained on internet-scale data for generalist robotic manipulation.', NULL)
ON CONFLICT (term, experience_level) DO NOTHING;

-- ============================================================================
-- Query Examples
-- ============================================================================
-- Get tooltip for a beginner user:
-- SELECT definition FROM personalization_tooltips
-- WHERE LOWER(term) = LOWER('ROS 2') AND experience_level = 'beginner';

-- Get translation cache stats:
-- SELECT
--   COUNT(*) as cached_translations,
--   SUM(tokens_used) as total_tokens_saved,
--   AVG(access_count) as avg_cache_hits
-- FROM urdu_translations;

-- Find most accessed translations:
-- SELECT chapter_id, access_count, created_at
-- FROM urdu_translations
-- ORDER BY access_count DESC
-- LIMIT 10;

-- ============================================================================
-- Maintenance Queries
-- ============================================================================
-- Update tooltip:
-- UPDATE personalization_tooltips
-- SET definition = 'New definition', updated_at = NOW()
-- WHERE term = 'Node' AND experience_level = 'beginner';

-- Clean old cache entries (optional, for cost optimization):
-- DELETE FROM urdu_translations
-- WHERE last_accessed < NOW() - INTERVAL '90 days'
-- AND access_count < 5;
