/**
 * RDK Device Connection API
 * 
 * Functions for testing and monitoring RDK X5 device connectivity
 */

export interface RDKConnectionTestResult {
  reachable: boolean;
  sshAvailable: boolean;
  serviceHealthy: boolean;
  connected: boolean;
  status: 'connected' | 'ssh_failed' | 'unreachable' | 'unknown';
  message: string;
  host: string;
  timestamp: string;
}

export interface RDKStatusResult {
  statusCheckAvailable: boolean;
  message: string;
  timestamp: string;
}

/**
 * Test connection to RDK device
 * Performs network ping, SSH check, and service health check
 * 
 * @param host - RDK IP address or hostname
 * @param user - SSH username (default: 'root')
 * @returns Connection test results
 */
export async function testRDKConnection(
  host: string, 
  user: string = 'root'
): Promise<RDKConnectionTestResult> {
  const response = await fetch('/api/rdk/test-connection', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ host, user })
  });
  
  if (!response.ok) {
    const error = await response.json().catch(() => ({ message: 'Connection test failed' }));
    throw new Error(error.message || 'Failed to test RDK connection');
  }
  
  return await response.json();
}

/**
 * Get current RDK status information
 * @returns Status information
 */
export async function getRDKStatus(): Promise<RDKStatusResult> {
  const response = await fetch('/api/rdk/status');
  
  if (!response.ok) {
    throw new Error('Failed to get RDK status');
  }
  
  return await response.json();
}
