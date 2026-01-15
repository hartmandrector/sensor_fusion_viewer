/**
 * UI Elements Module
 * 
 * Centralized DOM element references.
 * Provides type-safe access to all UI elements used by the application.
 */

// ============================================================================
// Element Interface
// ============================================================================

/**
 * All DOM elements used by the application
 */
export interface UIElements {
  // File handling
  csvFile: HTMLInputElement;
  fileName: HTMLSpanElement;
  
  // Playback controls
  playBtn: HTMLButtonElement;
  pauseBtn: HTMLButtonElement;
  resetBtn: HTMLButtonElement;
  timeSlider: HTMLInputElement;
  currentTime: HTMLSpanElement;
  totalTime: HTMLSpanElement;
  speedSelect: HTMLSelectElement;
  
  // Algorithm selection
  algorithmSelect: HTMLSelectElement;
  fusionSettings: HTMLDivElement;
  
  // Filter parameters
  betaSlider: HTMLInputElement;
  betaValue: HTMLSpanElement;
  initFromSensors: HTMLInputElement;
  useMagnetometer: HTMLInputElement;
  showSensorVectors: HTMLInputElement;
  showLinearAccel: HTMLInputElement;
  showEarthAccel: HTMLInputElement;
  showGravity: HTMLInputElement;
  showHeading: HTMLInputElement;
  showCompassHeading: HTMLInputElement;
  
  // Fusion Ch.7 specific
  accelRejectSlider: HTMLInputElement;
  accelRejectValue: HTMLSpanElement;
  magRejectSlider: HTMLInputElement;
  magRejectValue: HTMLSpanElement;
  accelStatus: HTMLSpanElement;
  accelError: HTMLSpanElement;
  magStatus: HTMLSpanElement;
  magError: HTMLSpanElement;
  ahrsFlags: HTMLSpanElement;
  // Runtime bias display
  runtimeBiasX: HTMLSpanElement;
  runtimeBiasY: HTMLSpanElement;
  runtimeBiasZ: HTMLSpanElement;
  biasCalStatus: HTMLSpanElement;
  biasProgress: HTMLSpanElement;
  gyroMagnitude: HTMLSpanElement;
  stationaryThreshold: HTMLSpanElement;
  
  // Axis remapping
  imuRemapX: HTMLSelectElement;
  imuRemapY: HTMLSelectElement;
  imuRemapZ: HTMLSelectElement;
  magRemapX: HTMLSelectElement;
  magRemapY: HTMLSelectElement;
  magRemapZ: HTMLSelectElement;
  
  // Diagnostic displays
  rawAccelDisplay: HTMLSpanElement;
  rawMagDisplay: HTMLSpanElement;
  
  // Magnetometer calibration
  magOffsetX: HTMLInputElement;
  magOffsetY: HTMLInputElement;
  magOffsetZ: HTMLInputElement;
  calcCalibrationBtn: HTMLButtonElement;
  showMagPlotBtn: HTMLButtonElement;
  calibrationResult: HTMLDivElement;
  
  // Orientation display
  heading: HTMLSpanElement;
  pitch: HTMLSpanElement;
  roll: HTMLSpanElement;
  qw: HTMLSpanElement;
  qx: HTMLSpanElement;
  qy: HTMLSpanElement;
  qz: HTMLSpanElement;
  
  // Raw sensor display
  gyroX: HTMLSpanElement;
  gyroY: HTMLSpanElement;
  gyroZ: HTMLSpanElement;
  accelX: HTMLSpanElement;
  accelY: HTMLSpanElement;
  accelZ: HTMLSpanElement;
  magX: HTMLSpanElement;
  magY: HTMLSpanElement;
  magZ: HTMLSpanElement;
  
  // Dataset stats
  imuCount: HTMLSpanElement;
  magCount: HTMLSpanElement;
  duration: HTMLSpanElement;
  imuRate: HTMLSpanElement;
  magRate: HTMLSpanElement;
  
  // IMU calibration
  gyroBiasX: HTMLInputElement;
  gyroBiasY: HTMLInputElement;
  gyroBiasZ: HTMLInputElement;
  accelOffsetX: HTMLInputElement;
  accelOffsetY: HTMLInputElement;
  accelOffsetZ: HTMLInputElement;
  calcIMUCalBtn: HTMLButtonElement;
  calc6PosCalBtn: HTMLButtonElement;
  analyzeIMUBtn: HTMLButtonElement;
  showIMUPlotBtn: HTMLButtonElement;
  imuCalibrationResult: HTMLDivElement;
  accelScaleMatrixDisplay: HTMLDivElement;
  sixPosStatus: HTMLDivElement;
  orientationStatus: HTMLDivElement;
  
  // Mag calibration - ellipsoid
  calcEllipsoidBtn: HTMLButtonElement;
  softIronMatrixDisplay: HTMLDivElement;
  
  // Export
  exportFusedDataBtn: HTMLButtonElement;
  exportStatus: HTMLDivElement;
  
  // Acceleration Integration
  integrationStartSlider: HTMLInputElement;
  integrationStartTime: HTMLSpanElement;
  calculateIntegrationBtn: HTMLButtonElement;
  showChartsBtn: HTMLButtonElement;
  
  // Charts panel
  chartsPanel: HTMLDivElement;
  viewerPanel: HTMLDivElement;
  componentSelect: HTMLSelectElement;
  backToViewerBtn: HTMLButtonElement;
  componentChartCanvas: HTMLCanvasElement;
  topDownChartCanvas: HTMLCanvasElement;
  profileChartCanvas: HTMLCanvasElement;
  speedChartCanvas: HTMLCanvasElement;
}

// ============================================================================
// Element Getter
// ============================================================================

/**
 * Get an element by ID with type casting
 */
function getElement<T extends HTMLElement>(id: string): T {
  const element = document.getElementById(id);
  if (!element) {
    throw new Error(`Element with id '${id}' not found`);
  }
  return element as T;
}

/**
 * Initialize and return all UI element references
 * Call this after DOM is ready
 */
export function initializeElements(): UIElements {
  return {
    // File handling
    csvFile: getElement<HTMLInputElement>('csvFile'),
    fileName: getElement<HTMLSpanElement>('fileName'),
    
    // Playback controls
    playBtn: getElement<HTMLButtonElement>('playBtn'),
    pauseBtn: getElement<HTMLButtonElement>('pauseBtn'),
    resetBtn: getElement<HTMLButtonElement>('resetBtn'),
    timeSlider: getElement<HTMLInputElement>('timeSlider'),
    currentTime: getElement<HTMLSpanElement>('currentTime'),
    totalTime: getElement<HTMLSpanElement>('totalTime'),
    speedSelect: getElement<HTMLSelectElement>('speedSelect'),
    
    // Algorithm selection
    algorithmSelect: getElement<HTMLSelectElement>('algorithmSelect'),
    fusionSettings: getElement<HTMLDivElement>('fusionSettings'),
    
    // Filter parameters
    betaSlider: getElement<HTMLInputElement>('betaSlider'),
    betaValue: getElement<HTMLSpanElement>('betaValue'),
    initFromSensors: getElement<HTMLInputElement>('initFromSensors'),
    useMagnetometer: getElement<HTMLInputElement>('useMagnetometer'),
    showSensorVectors: getElement<HTMLInputElement>('showSensorVectors'),
    showLinearAccel: getElement<HTMLInputElement>('showLinearAccel'),
    showEarthAccel: getElement<HTMLInputElement>('showEarthAccel'),
    showGravity: getElement<HTMLInputElement>('showGravity'),
    showHeading: getElement<HTMLInputElement>('showHeading'),
    showCompassHeading: getElement<HTMLInputElement>('showCompassHeading'),
    
    // Fusion Ch.7 specific
    accelRejectSlider: getElement<HTMLInputElement>('accelRejectSlider'),
    accelRejectValue: getElement<HTMLSpanElement>('accelRejectValue'),
    magRejectSlider: getElement<HTMLInputElement>('magRejectSlider'),
    magRejectValue: getElement<HTMLSpanElement>('magRejectValue'),
    accelStatus: getElement<HTMLSpanElement>('accelStatus'),
    accelError: getElement<HTMLSpanElement>('accelError'),
    magStatus: getElement<HTMLSpanElement>('magStatus'),
    magError: getElement<HTMLSpanElement>('magError'),
    ahrsFlags: getElement<HTMLSpanElement>('ahrsFlags'),
    // Runtime bias display
    runtimeBiasX: getElement<HTMLSpanElement>('runtimeBiasX'),
    runtimeBiasY: getElement<HTMLSpanElement>('runtimeBiasY'),
    runtimeBiasZ: getElement<HTMLSpanElement>('runtimeBiasZ'),
    biasCalStatus: getElement<HTMLSpanElement>('biasCalStatus'),
    biasProgress: getElement<HTMLSpanElement>('biasProgress'),
    gyroMagnitude: getElement<HTMLSpanElement>('gyroMagnitude'),
    stationaryThreshold: getElement<HTMLSpanElement>('stationaryThreshold'),
    
    // Axis remapping
    imuRemapX: getElement<HTMLSelectElement>('imuRemapX'),
    imuRemapY: getElement<HTMLSelectElement>('imuRemapY'),
    imuRemapZ: getElement<HTMLSelectElement>('imuRemapZ'),
    magRemapX: getElement<HTMLSelectElement>('magRemapX'),
    magRemapY: getElement<HTMLSelectElement>('magRemapY'),
    magRemapZ: getElement<HTMLSelectElement>('magRemapZ'),
    
    // Diagnostic displays
    rawAccelDisplay: getElement<HTMLSpanElement>('rawAccelDisplay'),
    rawMagDisplay: getElement<HTMLSpanElement>('rawMagDisplay'),
    
    // Magnetometer calibration
    magOffsetX: getElement<HTMLInputElement>('magOffsetX'),
    magOffsetY: getElement<HTMLInputElement>('magOffsetY'),
    magOffsetZ: getElement<HTMLInputElement>('magOffsetZ'),
    calcCalibrationBtn: getElement<HTMLButtonElement>('calcCalibrationBtn'),
    showMagPlotBtn: getElement<HTMLButtonElement>('showMagPlotBtn'),
    calibrationResult: getElement<HTMLDivElement>('calibrationResult'),
    
    // Orientation display
    heading: getElement<HTMLSpanElement>('heading'),
    pitch: getElement<HTMLSpanElement>('pitch'),
    roll: getElement<HTMLSpanElement>('roll'),
    qw: getElement<HTMLSpanElement>('qw'),
    qx: getElement<HTMLSpanElement>('qx'),
    qy: getElement<HTMLSpanElement>('qy'),
    qz: getElement<HTMLSpanElement>('qz'),
    
    // Raw sensor display
    gyroX: getElement<HTMLSpanElement>('gyroX'),
    gyroY: getElement<HTMLSpanElement>('gyroY'),
    gyroZ: getElement<HTMLSpanElement>('gyroZ'),
    accelX: getElement<HTMLSpanElement>('accelX'),
    accelY: getElement<HTMLSpanElement>('accelY'),
    accelZ: getElement<HTMLSpanElement>('accelZ'),
    magX: getElement<HTMLSpanElement>('magX'),
    magY: getElement<HTMLSpanElement>('magY'),
    magZ: getElement<HTMLSpanElement>('magZ'),
    
    // Dataset stats
    imuCount: getElement<HTMLSpanElement>('imuCount'),
    magCount: getElement<HTMLSpanElement>('magCount'),
    duration: getElement<HTMLSpanElement>('duration'),
    imuRate: getElement<HTMLSpanElement>('imuRate'),
    magRate: getElement<HTMLSpanElement>('magRate'),
    
    // IMU calibration
    gyroBiasX: getElement<HTMLInputElement>('gyroBiasX'),
    gyroBiasY: getElement<HTMLInputElement>('gyroBiasY'),
    gyroBiasZ: getElement<HTMLInputElement>('gyroBiasZ'),
    accelOffsetX: getElement<HTMLInputElement>('accelOffsetX'),
    accelOffsetY: getElement<HTMLInputElement>('accelOffsetY'),
    accelOffsetZ: getElement<HTMLInputElement>('accelOffsetZ'),
    calcIMUCalBtn: getElement<HTMLButtonElement>('calcIMUCalBtn'),
    calc6PosCalBtn: getElement<HTMLButtonElement>('calc6PosCalBtn'),
    analyzeIMUBtn: getElement<HTMLButtonElement>('analyzeIMUBtn'),
    showIMUPlotBtn: getElement<HTMLButtonElement>('showIMUPlotBtn'),
    imuCalibrationResult: getElement<HTMLDivElement>('imuCalibrationResult'),
    accelScaleMatrixDisplay: getElement<HTMLDivElement>('accelScaleMatrixDisplay'),
    sixPosStatus: getElement<HTMLDivElement>('sixPosStatus'),
    orientationStatus: getElement<HTMLDivElement>('orientationStatus'),
    
    // Mag calibration - ellipsoid
    calcEllipsoidBtn: getElement<HTMLButtonElement>('calcEllipsoidBtn'),
    softIronMatrixDisplay: getElement<HTMLDivElement>('softIronMatrixDisplay'),
    
    // Export
    exportFusedDataBtn: getElement<HTMLButtonElement>('exportFusedDataBtn'),
    exportStatus: getElement<HTMLDivElement>('exportStatus'),
    
    // Acceleration Integration
    integrationStartSlider: getElement<HTMLInputElement>('integrationStartSlider'),
    integrationStartTime: getElement<HTMLSpanElement>('integrationStartTime'),
    calculateIntegrationBtn: getElement<HTMLButtonElement>('calculateIntegrationBtn'),
    showChartsBtn: getElement<HTMLButtonElement>('showChartsBtn'),
    
    // Charts panel
    chartsPanel: getElement<HTMLDivElement>('chartsPanel'),
    viewerPanel: getElement<HTMLDivElement>('viewerPanel'),
    componentSelect: getElement<HTMLSelectElement>('componentSelect'),
    backToViewerBtn: getElement<HTMLButtonElement>('backToViewerBtn'),
    componentChartCanvas: getElement<HTMLCanvasElement>('componentChartCanvas'),
    topDownChartCanvas: getElement<HTMLCanvasElement>('topDownChartCanvas'),
    profileChartCanvas: getElement<HTMLCanvasElement>('profileChartCanvas'),
    speedChartCanvas: getElement<HTMLCanvasElement>('speedChartCanvas'),
  };
}

// ============================================================================
// Singleton Elements Reference
// ============================================================================

let _elements: UIElements | null = null;

/**
 * Get the UI elements (initializes on first call)
 */
export function getElements(): UIElements {
  if (!_elements) {
    _elements = initializeElements();
  }
  return _elements;
}
