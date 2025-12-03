#!/bin/bash

# Script to restart frontend server and clear cache

echo "🔄 Restarting frontend server..."

# Navigate to frontend directory
cd frontend || exit 1

# Clear Docusaurus build cache
echo "🗑️  Clearing build cache..."
rm -rf .docusaurus build

# Clear node_modules cache (optional, but helps)
echo "🧹 Clearing node cache..."
rm -rf node_modules/.cache

echo "✅ Cache cleared!"
echo ""
echo "📦 Starting frontend server..."
echo "   (Wait for 'webpack compiled successfully' message)"
echo ""

# Start the server
npm start

