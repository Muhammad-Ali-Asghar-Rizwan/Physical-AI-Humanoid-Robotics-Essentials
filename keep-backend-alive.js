// Simple script to ping the backend periodically to keep it awake
// This can be run as a cron job or scheduled task to ping every few minutes

async function pingBackend() {
  const backendUrl = process.env.BACKEND_API_URL || 'https://physical-ai-humanoid-robotics-essentials-production.up.railway.app';
  
  try {
    const response = await fetch(`${backendUrl}/health`);
    if (response.ok) {
      console.log(`[${new Date().toISOString()}] Backend is awake and healthy`);
    } else {
      console.log(`[${new Date().toISOString()}] Backend responded with status: ${response.status}`);
    }
  } catch (error) {
    console.error(`[${new Date().toISOString()}] Error pinging backend:`, error.message);
  }
}

// Run immediately when script is executed
pingBackend();

// Ping every 10 minutes to keep the backend awake (for Railway free tier)
setInterval(pingBackend, 10 * 60 * 1000); // 10 minutes