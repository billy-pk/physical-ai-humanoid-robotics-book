# Deployment Guide for GitHub Pages

This guide explains how to deploy the Physical AI & Humanoid Robotics Book to GitHub Pages.

## Prerequisites

1. **Git configured** with your GitHub credentials
2. **Node.js 20+** installed
3. **GitHub repository** access: `billy-pk/physical-ai-humanoid-robotics-book`

## Quick Deployment

### Option 1: Using Docusaurus Deploy Command (Recommended)

```bash
cd frontend
npm run build
GIT_USER=billy-pk USE_SSH=true npm run deploy
```

This will:
1. Build the site
2. Create/update the `gh-pages` branch
3. Push to GitHub
4. GitHub Pages will automatically update

### Option 2: Manual Deployment

```bash
# 1. Build the site
cd frontend
npm run build

# 2. Navigate to build directory
cd build

# 3. Initialize git (if not already)
git init

# 4. Add and commit
git add .
git commit -m "Deploy updated book content"

# 5. Add remote (if not exists)
git remote add origin https://github.com/billy-pk/physical-ai-humanoid-robotics-book.git

# 6. Push to gh-pages branch
git branch -M gh-pages
git push -f origin gh-pages
```

## Verification

After deployment, check:
- https://billy-pk.github.io/physical-ai-humanoid-robotics-book/

The site should update within 1-2 minutes.

## Troubleshooting

### Issue: "Permission denied"
**Solution**: Use SSH or configure GitHub token
```bash
# Use SSH
GIT_USER=billy-pk USE_SSH=true npm run deploy

# Or configure GitHub token
export GITHUB_TOKEN=your_token_here
```

### Issue: "gh-pages branch not found"
**Solution**: The deploy command will create it automatically

### Issue: "Build fails"
**Solution**: Check for broken links or markdown errors
```bash
npm run build 2>&1 | grep -i error
```

## Automated Deployment (GitHub Actions)

For automatic deployment on push to main, see `.github/workflows/deploy.yml` (if exists).
