// Temporary proxy solution for CORS issues
// This should only be used temporarily until the main backend is fixed

const BACKEND_API_URL = process.env.REACT_APP_BACKEND_URL || 
  (typeof process !== 'undefined' && process.env && process.env.NODE_ENV === 'development' 
   ? 'http://localhost:8000' 
   : 'https://physical-ai-humanoid-robotics-essentials-production.up.railway.app/');

// Use a CORS proxy as a temporary solution
const CORS_PROXY = 'https://cors-anywhere.herokuapp.com/';
const PROXIED_BACKEND_URL = `${CORS_PROXY}${BACKEND_API_URL}`;

// Function to get the appropriate backend URL based on environment
function getBackendUrl(useProxy = false) {
  if (useProxy) {
    return PROXIED_BACKEND_URL;
  }
  return BACKEND_API_URL;
}

export { getBackendUrl, BACKEND_API_URL };