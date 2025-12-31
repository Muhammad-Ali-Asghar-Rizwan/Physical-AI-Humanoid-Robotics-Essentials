# Fix for "Failed to fetch" Error in Deployed Version

## Problem
The chatbot works fine locally but throws "Error: Failed to fetch" when deployed to Vercel.

## Root Causes & Solutions

### 1. CORS Configuration Issue
The backend was not configured to accept requests from your new Vercel deployment URL.

**Fix Applied:**
- Added `https://physical-ai-humanoid-robotics-essen-opal.vercel.app` to the allowed origins in the backend's CORS configuration.

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

## Deployment Steps

### Backend (Railway)
1. Commit and push the updated backend code with the new CORS configuration
2. Redeploy your Railway backend

### Frontend (Vercel)
1. Commit and push the updated frontend code with the fixed configuration
2. Redeploy your Vercel frontend

## Optional: Keep Backend Awake
To prevent the Railway backend from sleeping (which causes slow first responses), you can run the keep-alive script:

```bash
node keep-backend-alive.js
```

Or set up a cron job or scheduled task to periodically ping the backend's health endpoint.

## Testing
After redeployment, the chatbot should work correctly on your Vercel deployment without the "Failed to fetch" error.