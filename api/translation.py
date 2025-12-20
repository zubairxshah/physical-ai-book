# Vercel serverless wrapper for translation API

import sys
import os

# Add backend directory to Python path
backend_path = os.path.join(os.path.dirname(__file__), '..', 'backend')
sys.path.insert(0, backend_path)

from translation_api import app

# Export handler for Vercel
handler = app
