/**
 * GPS UI Handler Module
 * 
 * Handles UI interactions for GPS/Track data loading and integration.
 * Keeps GPS-related UI logic separate from main.ts.
 */

import { loadTrackCSV, gpsState, updateGPSIntegration, storeRawSensorCSV } from './gpsIntegration';
import { state } from './appState';

// ============================================================================
// UI Element References
// ============================================================================

let trackFileInput: HTMLInputElement | null = null;
let loadTrackBtn: HTMLButtonElement | null = null;
let gpsStatusSpan: HTMLSpanElement | null = null;

// ============================================================================
// Initialization
// ============================================================================

/**
 * Initialize GPS UI elements
 * Call this after DOM is ready
 */
export function initializeGPSUI(): void {
  trackFileInput = document.getElementById('trackFile') as HTMLInputElement;
  loadTrackBtn = document.getElementById('loadTrackBtn') as HTMLButtonElement;
  gpsStatusSpan = document.getElementById('gpsStatus') as HTMLSpanElement;
  
  if (trackFileInput) {
    trackFileInput.addEventListener('change', handleTrackFileSelect);
  }
  
  if (loadTrackBtn) {
    loadTrackBtn.addEventListener('click', () => {
      trackFileInput?.click();
    });
  }
}

/**
 * Enable GPS UI controls (call after sensor file is loaded)
 */
export function enableGPSControls(): void {
  if (loadTrackBtn) {
    loadTrackBtn.disabled = false;
  }
}

/**
 * Disable GPS UI controls
 */
export function disableGPSControls(): void {
  if (loadTrackBtn) {
    loadTrackBtn.disabled = true;
  }
}

/**
 * Update GPS status display
 */
export function updateGPSStatus(message: string, isError: boolean = false): void {
  if (gpsStatusSpan) {
    gpsStatusSpan.textContent = message;
    gpsStatusSpan.style.color = isError ? '#ff6666' : '#88ff88';
  }
}

// ============================================================================
// Event Handlers
// ============================================================================

/**
 * Handle track file selection
 */
async function handleTrackFileSelect(event: Event): Promise<void> {
  const input = event.target as HTMLInputElement;
  const file = input.files?.[0];
  
  if (!file) return;
  
  updateGPSStatus('Loading...', false);
  
  const result = await loadTrackCSV(file);
  
  if (result.success) {
    updateGPSStatus(result.message, false);
    
    // If charts are visible, trigger a chart update
    const chartsPanel = document.getElementById('chartsPanel');
    if (chartsPanel && window.getComputedStyle(chartsPanel).display !== 'none') {
      // Trigger chart refresh by dispatching a custom event
      window.dispatchEvent(new CustomEvent('gps-data-loaded'));
    }
  } else {
    updateGPSStatus(result.message, true);
  }
  
  // Reset file input so same file can be selected again
  input.value = '';
}

/**
 * Called when sensor file is loaded to store raw content for $TIME parsing
 */
export function onSensorFileLoaded(content: string): void {
  storeRawSensorCSV(content);
  enableGPSControls();
  updateGPSStatus('Ready to load TRACK.CSV', false);
}

/**
 * Called when integration start time changes
 * Updates GPS integration to use new start time
 */
export function onIntegrationStartTimeChange(): void {
  if (gpsState.isLoaded) {
    // Get current start time from slider
    const slider = document.getElementById('integrationStartSlider') as HTMLInputElement;
    if (slider) {
      const relativeTime = parseFloat(slider.value);
      const datasetStartTime = parseFloat(slider.dataset.startTime || '0');
      state.integrationStartTime = datasetStartTime + relativeTime;
      
      updateGPSIntegration();
    }
  }
}

// ============================================================================
// Exports
// ============================================================================

export { gpsState, hasGPSData } from './gpsIntegration';
