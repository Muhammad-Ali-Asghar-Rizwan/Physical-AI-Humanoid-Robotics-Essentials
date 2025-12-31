// Test script to verify backend connectivity
async function testBackendConnection() {
  const backendUrl = 'https://physical-ai-humanoid-robotics-essentials-production.up.railway.app';
  
  console.log('Testing backend connectivity...');
  console.log('Target URL:', backendUrl);
  
  try {
    // Test health endpoint first
    console.log('\n1. Testing health endpoint...');
    const healthResponse = await fetch(`${backendUrl}/health`);
    console.log('Health endpoint status:', healthResponse.status);
    
    if (healthResponse.ok) {
      const healthData = await healthResponse.json();
      console.log('Health response:', healthData);
    } else {
      console.log('Health endpoint returned error:', await healthResponse.text().catch(() => 'Could not read response'));
    }
    
    // Test root endpoint
    console.log('\n2. Testing root endpoint...');
    const rootResponse = await fetch(`${backendUrl}/`);
    console.log('Root endpoint status:', rootResponse.status);
    
    if (rootResponse.ok) {
      const rootData = await rootResponse.json();
      console.log('Root response:', rootData);
    } else {
      console.log('Root endpoint returned error:', await rootResponse.text().catch(() => 'Could not read response'));
    }
    
    // Test CORS headers by making a request with appropriate headers
    console.log('\n3. Testing CORS by simulating frontend request...');
    const corsTestResponse = await fetch(`${backendUrl}/health`, {
      method: 'GET',
      headers: {
        'Origin': 'https://physical-ai-humanoid-robotics-essen-opal.vercel.app',
        'Content-Type': 'application/json',
      }
    });
    
    console.log('CORS test status:', corsTestResponse.status);
    console.log('Access-Control-Allow-Origin header:', corsTestResponse.headers.get('access-control-allow-origin'));
    
  } catch (error) {
    console.error('\nError during backend connectivity test:', error.message);
  }
}

// Run the test
testBackendConnection();