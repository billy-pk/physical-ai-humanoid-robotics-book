# Frontend URLs Guide

## Development URLs

Due to the `baseUrl` configuration in `docusaurus.config.ts`, all pages are served with the `/physical-ai-humanoid-robotics-book/` prefix:

### Main Pages
- **Home**: http://localhost:3000/physical-ai-humanoid-robotics-book/
- **Signup**: http://localhost:3000/physical-ai-humanoid-robotics-book/signup
- **Sign In**: http://localhost:3000/physical-ai-humanoid-robotics-book/signin
- **Docs**: http://localhost:3000/physical-ai-humanoid-robotics-book/docs/intro

### Why the baseUrl?

The `baseUrl: '/physical-ai-humanoid-robotics-book/'` is configured for GitHub Pages deployment. GitHub Pages serves your site at:
```
https://billy-pk.github.io/physical-ai-humanoid-robotics-book/
```

So the baseUrl matches the repository name to ensure proper routing in production.

## Quick Access

**For Testing Authentication**:
1. Open: http://localhost:3000/physical-ai-humanoid-robotics-book/signup
2. Complete signup form
3. Fill questionnaire
4. Verify success

## Production URLs

When deployed to GitHub Pages:
- **Home**: https://billy-pk.github.io/physical-ai-humanoid-robotics-book/
- **Signup**: https://billy-pk.github.io/physical-ai-humanoid-robotics-book/signup
- **Sign In**: https://billy-pk.github.io/physical-ai-humanoid-robotics-book/signin

## Note

If you want to remove the baseUrl prefix in development (for easier testing), you can temporarily change `baseUrl` to `/` in `docusaurus.config.ts`, but remember to change it back before deploying to GitHub Pages.
