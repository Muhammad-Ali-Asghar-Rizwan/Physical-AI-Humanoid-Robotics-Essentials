# Fix for "Failed to fetch" and 404 Error in Deployed Version

## Problem
The chatbot works fine locally but throws "Error: Failed to fetch", "Unable to connect to the backend. The server might be temporarily unavailable. Please try again in a moment.", or "Backend returned 404: {"detail":"Not Found"}" when deployed to Vercel.

## Root Causes & Solutions

### 1. CORS Configuration Issue
The backend was not configured to accept requests from your new Vercel deployment URL.

**Fix Applied:**
- Added `https://physical-ai-humanoid-robotics-essen-opal.vercel.app` to the allowed origins in the backend's CORS configuration.
- Added specific OPTIONS endpoints for preflight requests
- Enhanced CORS headers to be more specific

### 2. Configuration Field Mismatch
The frontend was looking for a configuration field named `backendApiUrl`, but the docusaurus config had it as `backendUrl`.

**Fix Applied:**
- Changed the field name in `docusaurus.config.js` from `backendUrl` to `backendApiUrl` to match what the chatbot component expects.

### 3. Potential Backend Sleep Issue
Railway free tier dynos go to sleep after inactivity, which can cause timeout issues.

**Fix Applied:**
- Added timeout handling (30 seconds) with AbortController in the frontend
- Improved error messages to help diagnose sleep/wake issues
- Created a keep-alive script to ping the backend periodically

### 4. Backend Startup/Dependency Issues
The backend might have issues starting up or connecting to required services (Cohere, Qdrant, etc.) in the Railway environment, causing routes to not be properly registered.

**Fix Applied:**
- Added error handling for missing environment variables
- Added logging to help debug startup issues
- Added checks to ensure required services are available before processing requests

## Deployment Steps

### Backend (Railway)
1. Commit and push the updated backend code with all the fixes
2. Verify all required environment variables are set in Railway dashboard:
   - COHERE_API_KEY
   - QDRANT_API_KEY
   - QDRANT_URL
   - OPENAI_API_KEY (if needed)
3. Redeploy your Railway backend
4. Check Railway logs for successful startup and any errors

### Frontend (Vercel)
1. Commit and push the updated frontend code with the fixed configuration
2. Redeploy your Vercel frontend

## Troubleshooting Steps

### 1. Check Backend Status
Run the test script to verify backend connectivity:
```bash
node test-backend-connection.js
```

### 2. Check Railway Logs
1. Go to your Railway dashboard
2. Select your backend project
3. Check the logs for any error messages during startup or when receiving requests
4. Look for the "Application startup complete" message to confirm successful initialization

### 3. Verify Environment Variables
Make sure these environment variables are set in your Railway dashboard:
- COHERE_API_KEY
- QDRANT_API_KEY
- QDRANT_URL
- OPENAI_API_KEY (if needed)

### 4. Test Backend Endpoints Directly
Try accessing these endpoints directly in your browser or using curl:
- `https://physical-ai-humanoid-robotics-essentials-production.up.railway.app/health`
- `https://physical-ai-humanoid-robotics-essentials-production.up.railway.app/`

## Testing
After redeployment and verification, the chatbot should work correctly on your Vercel deployment without the connection errors.