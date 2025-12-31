# Troubleshooting CORS Issues

## Problem
The backend works when accessed directly, but the frontend gets "Unable to connect to the backend" error when making requests.

## Root Cause
This is a classic CORS (Cross-Origin Resource Sharing) issue. Even though the backend is accessible directly in the browser, browsers enforce CORS policies when JavaScript code running on one domain (your Vercel frontend) tries to make requests to another domain (your Railway backend).

## What We've Fixed
1. Added your Vercel URL to the allowed origins in the backend CORS configuration
2. Added specific OPTIONS endpoints for preflight requests
3. Enhanced CORS headers to be more specific
4. Added logging to help debug issues

## Next Steps
1. Redeploy your backend to Railway with these changes
2. After deployment, check the Railway logs to see if requests are coming through
3. Open browser developer tools (F12) and check the Network tab for specific error messages

## Checking Browser Console
When you try to use the chatbot, look for these specific errors in the browser console:
- "Access to fetch at 'https://...' from origin 'https://...' has been blocked by CORS policy"
- "Response to preflight request doesn't pass access control check"
- "Request header field content-type is not allowed by Access-Control-Allow-Headers"

## Testing After Backend Redeployment
1. First, make sure your backend is properly deployed with the new changes
2. Check Railway logs for successful startup
3. Then test the frontend again

## Additional Notes
- The OPTIONS endpoints we added specifically handle preflight requests that browsers send before the actual POST request
- The CORS headers are now more specific to your domain rather than using wildcards
- Logging has been added to help identify any remaining issues