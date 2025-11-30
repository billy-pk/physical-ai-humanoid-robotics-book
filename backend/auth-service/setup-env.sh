#!/bin/bash
# Setup script for Better Auth service environment

set -e

echo "🔧 Setting up Better Auth service environment..."

# Check if .env already exists
if [ -f .env ]; then
    echo "⚠️  .env file already exists. Backing up to .env.backup"
    cp .env .env.backup
fi

# Copy example file
cp .env.example .env

# Generate a new secret if needed
if grep -q "your-secret-key-here" .env || grep -q "c7ea42c8b5e5f5f01b30e091a29fef8bcaa490b8572e7f421216434911347dc4" .env; then
    echo "🔑 Generating new BETTER_AUTH_SECRET..."
    NEW_SECRET=$(node -e "console.log(require('crypto').randomBytes(32).toString('hex'))")
    if [[ "$OSTYPE" == "darwin"* ]]; then
        # macOS
        sed -i '' "s/BETTER_AUTH_SECRET=.*/BETTER_AUTH_SECRET=\"$NEW_SECRET\"/" .env
    else
        # Linux
        sed -i "s/BETTER_AUTH_SECRET=.*/BETTER_AUTH_SECRET=\"$NEW_SECRET\"/" .env
    fi
    echo "✅ Generated new secret"
fi

echo ""
echo "✅ Environment file created: .env"
echo ""
echo "📝 Next steps:"
echo "   1. Review and update .env with your actual values:"
echo "      - DATABASE_URL: Copy from backend/.env (NEON_DATABASE_URL)"
echo "      - BETTER_AUTH_URL: Set to your production URL when deploying"
echo "      - BETTER_AUTH_TRUSTED_ORIGINS: Add your frontend domain(s)"
echo ""
echo "   2. Start the service:"
echo "      npm run dev"
echo ""
