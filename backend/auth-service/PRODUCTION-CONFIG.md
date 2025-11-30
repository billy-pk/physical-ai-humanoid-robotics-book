# Production Configuration Guide

## GitHub Pages Domain

Your frontend is deployed at: **https://billy-pk.github.io/physical-ai-humanoid-robotics-book/**

## Environment Variables for Production

When deploying the Better Auth service to production (e.g., Render.com, Railway, etc.), update these environment variables:

### Required Variables

```env
# Database (same as development - Neon Postgres)
DATABASE_URL="postgresql://neondb_owner:npg_K3pdiU0IwGcM@ep-blue-thunder-a16pea4d-pooler.ap-southeast-1.aws.neon.tech/neondb?sslmode=require&channel_binding=require"

# Generate a NEW secret for production (don't use development secret)
BETTER_AUTH_SECRET="<generate-new-secret-here>"

# Production URL of your auth service (e.g., Render.com URL)
BETTER_AUTH_URL="https://your-auth-service.onrender.com"

# Trusted origins - include both development and production frontend URLs
BETTER_AUTH_TRUSTED_ORIGINS="http://localhost:3000,https://billy-pk.github.io"

# Port (usually set automatically by hosting platform)
PORT=3001
```

## Important Notes

1. **BETTER_AUTH_SECRET**: Generate a NEW secret for production:
   ```bash
   node -e "console.log(require('crypto').randomBytes(32).toString('hex'))"
   ```

2. **BETTER_AUTH_URL**: This should be your production auth service URL (e.g., `https://your-app.onrender.com`)

3. **BETTER_AUTH_TRUSTED_ORIGINS**: Must include:
   - `http://localhost:3000` (for local development)
   - `https://billy-pk.github.io` (your GitHub Pages domain)

4. **Frontend Configuration**: Update `frontend/src/lib/auth.ts` to use production auth URL:
   ```typescript
   const BETTER_AUTH_URL = 
     process.env.NEXT_PUBLIC_BETTER_AUTH_URL || 
     (typeof window !== "undefined" && window.location.hostname === "billy-pk.github.io"
       ? "https://your-auth-service.onrender.com"
       : "http://localhost:3001");
   ```

## Deployment Checklist

- [ ] Generate new `BETTER_AUTH_SECRET` for production
- [ ] Set `BETTER_AUTH_URL` to production auth service URL
- [ ] Set `BETTER_AUTH_TRUSTED_ORIGINS` to include GitHub Pages domain
- [ ] Deploy auth service to hosting platform
- [ ] Update frontend auth client configuration
- [ ] Test authentication in production
- [ ] Verify CORS works correctly

## Testing Production Config

After deployment, test:

1. **Health Check**:
   ```bash
   curl https://your-auth-service.onrender.com/health
   ```

2. **Signup from Production Frontend**:
   - Visit: https://billy-pk.github.io/physical-ai-humanoid-robotics-book/signup
   - Try signing up
   - Verify it works

3. **Check Browser Console**:
   - Look for CORS errors
   - Verify cookies are being set
   - Check network requests succeed
