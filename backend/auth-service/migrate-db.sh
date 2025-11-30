#!/bin/bash
# Script to run Better Auth database migrations
# This creates the required tables (user, session, account, verification)

set -e

echo "🔧 Running Better Auth database migrations..."
echo ""

# Check if .env exists
if [ ! -f .env ]; then
    echo "❌ .env file not found!"
    echo "   Run: cp .env.example .env"
    echo "   Then update DATABASE_URL with your Neon Postgres connection string"
    exit 1
fi

# Run migration (non-interactive)
echo "y" | npx @better-auth/cli@latest migrate

echo ""
echo "✅ Migration completed!"
echo ""
echo "📊 Better Auth tables created:"
echo "   - user (user accounts)"
echo "   - session (active sessions)"
echo "   - account (OAuth accounts)"
echo "   - verification (email verification tokens)"
echo ""
echo "🚀 You can now start the server: npm run dev"
