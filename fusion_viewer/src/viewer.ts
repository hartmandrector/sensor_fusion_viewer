/**
 * Three.js 3D Viewer for Sensor Fusion Orientation
 * 
 * Displays a 3D model representing the FlySight device orientation
 * with reference frame indicators.
 */

import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import type { Quaternion } from './types';
import type { MAGData } from './csvParser';
import { COLORS, DEVICE_DIMENSIONS, SCREEN_INSET, SCREEN_DEPTH_OFFSET } from './constants';
import { createCurvedArrow, disposeCurvedArrow } from './curvedArrow';

export class OrientationViewer {
  private container: HTMLElement;
  private scene: THREE.Scene;
  private camera: THREE.PerspectiveCamera;
  private renderer: THREE.WebGLRenderer;
  private controls: OrbitControls;
  private deviceGroup: THREE.Group;
  private magPlotGroup: THREE.Group | null = null;
  private imuPlotGroup: THREE.Group | null = null;
  private animationId: number | null = null;
  private showingMagPlot: boolean = false;
  private showingIMUPlot: boolean = false;
  
  // Sensor vector visualization
  private sensorVectorsGroup: THREE.Group;
  private accelArrow: THREE.ArrowHelper | null = null;
  private accelLabel: THREE.Sprite | null = null;
  private magArrow: THREE.ArrowHelper | null = null;
  private magLabel: THREE.Sprite | null = null;
  private magWorldArrow: THREE.ArrowHelper | null = null;  // Mag in world frame
  private magWorldLabel: THREE.Sprite | null = null;
  private magWorldDirection: THREE.Vector3 | null = null;  // Store mag world direction for heading
  private showSensorVectors: boolean = false;
  
  // Linear and Earth acceleration vectors
  private linearAccelArrow: THREE.ArrowHelper | null = null;
  private linearAccelLabel: THREE.Sprite | null = null;
  private earthAccelArrow: THREE.ArrowHelper | null = null;
  private earthAccelLabel: THREE.Sprite | null = null;
  private showLinearAccel: boolean = false;
  private showEarthAccel: boolean = false;
  
  // AHRS gravity vector (estimated gravity direction in body frame)
  private gravityArrow: THREE.ArrowHelper | null = null;
  private gravityLabel: THREE.Sprite | null = null;
  private showGravity: boolean = false;
  
  // Heading vector (compass heading in world frame)
  private headingArrow: THREE.ArrowHelper | null = null;
  private headingLabel: THREE.Sprite | null = null;
  private showHeading: boolean = false;
  
  // Gyro rotation arrows (curved)
  private gyroArrowX: THREE.Group | null = null;
  private gyroArrowY: THREE.Group | null = null;
  private gyroArrowZ: THREE.Group | null = null;
  
  constructor(containerId: string) {
    const container = document.getElementById(containerId);
    if (!container) {
      throw new Error(`Container element '${containerId}' not found`);
    }
    this.container = container;
    
    // Initialize Three.js
    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color(COLORS.SCENE_BACKGROUND);
    
    // Camera setup
    const aspect = container.clientWidth / container.clientHeight;
    this.camera = new THREE.PerspectiveCamera(50, aspect, 0.1, 1000);
    this.camera.position.set(3, 2, 3);
    this.camera.lookAt(0, 0, 0);
    
    // Renderer setup
    this.renderer = new THREE.WebGLRenderer({ antialias: true });
    this.renderer.setSize(container.clientWidth, container.clientHeight);
    this.renderer.setPixelRatio(window.devicePixelRatio);
    container.appendChild(this.renderer.domElement);
    
    // OrbitControls for camera interaction
    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.enableDamping = true;
    this.controls.dampingFactor = 0.05;
    this.controls.minDistance = 2;
    this.controls.maxDistance = 15;
    
    // Device group (will be rotated)
    this.deviceGroup = new THREE.Group();
    this.scene.add(this.deviceGroup);
    
    // Sensor vectors group (attached to device, shows raw sensor readings)
    this.sensorVectorsGroup = new THREE.Group();
    this.deviceGroup.add(this.sensorVectorsGroup);
    
    // Create scene elements
    this.createDevice();
    this.createReferenceFrame();
    this.createLighting();
    this.createGrid();
    
    // Handle resize
    window.addEventListener('resize', () => this.onResize());
    
    // Start render loop
    this.animate();
  }
  
  /**
   * Create the FlySight device representation
   * 
   * Device orientation: Z out front face (into screen/-Z in Three.js when at identity)
   * Y toward LED/top (up/+Y in Three.js), X to right side (+X in Three.js)
   */
  private createDevice(): void {
    // Main body - rectangular prism representing FlySight
    // Real dimensions: 5cm x 5cm x 1.5cm (W x H x D), scaled for visualization
    const bodyGeometry = new THREE.BoxGeometry(
      DEVICE_DIMENSIONS.WIDTH,
      DEVICE_DIMENSIONS.HEIGHT,
      DEVICE_DIMENSIONS.DEPTH
    );
    const bodyMaterial = new THREE.MeshPhongMaterial({ 
      color: COLORS.DEVICE_BODY,
      specular: 0x444444,
      shininess: 30
    });
    const body = new THREE.Mesh(bodyGeometry, bodyMaterial);
    this.deviceGroup.add(body);
    
    // LED indicator (top, toward front face which is at -Z)
    const ledGeometry = new THREE.SphereGeometry(0.05, 16, 16);
    const ledMaterial = new THREE.MeshBasicMaterial({ color: 0x00ff00 });
    const led = new THREE.Mesh(ledGeometry, ledMaterial);
    led.position.set(0, DEVICE_DIMENSIONS.HEIGHT / 2 - 0.05, -DEVICE_DIMENSIONS.DEPTH / 2 - 0.01);
    this.deviceGroup.add(led);
    
    // USB port indicator (bottom, toward back which is at +Z)
    const usbGeometry = new THREE.BoxGeometry(0.15, 0.05, 0.1);
    const usbMaterial = new THREE.MeshPhongMaterial({ color: 0x444444 });
    const usb = new THREE.Mesh(usbGeometry, usbMaterial);
    usb.position.set(0, -DEVICE_DIMENSIONS.HEIGHT / 2, DEVICE_DIMENSIONS.DEPTH / 2 - 0.05);
    this.deviceGroup.add(usb);
    
    // Front face indicator (screen area) - on -Z face
    const screenWidth = DEVICE_DIMENSIONS.WIDTH - SCREEN_INSET * 2;
    const screenHeight = DEVICE_DIMENSIONS.HEIGHT - SCREEN_INSET * 2;
    const screenGeometry = new THREE.PlaneGeometry(screenWidth, screenHeight);
    const screenMaterial = new THREE.MeshPhongMaterial({ 
      color: COLORS.DEVICE_SCREEN,
      side: THREE.FrontSide
    });
    const screen = new THREE.Mesh(screenGeometry, screenMaterial);
    screen.position.set(0, 0, -DEVICE_DIMENSIONS.DEPTH / 2 - SCREEN_DEPTH_OFFSET);
    screen.rotation.y = Math.PI;  // Rotate to face -Z direction
    this.deviceGroup.add(screen);
    
    // Device axes showing SENSOR FRAME labels (X, Y, Z)
    // FlySight 2 physical device orientation:
    //   Device Z = out the FRONT face (toward screen/North when upright facing North)
    //   Device Y = toward LED/TOP (up when device is upright)
    //   Device X = to the LEFT side when looking at front face (West when facing North)
    //
    // Three.js coordinate system:
    //   Three.js Y = up
    //   Three.js -Z = forward (into screen, toward North marker)
    //   Three.js -X = left (West)
    //
    // Mapping: Device X → Three.js -X, Device Y → Three.js +Y, Device Z → Three.js -Z
    const axisLength = 1.0;
    
    // Device X axis (red) - maps to Three.js -X (left/West)
    const xAxis = this.createArrow(COLORS.AXIS_X, axisLength);
    xAxis.rotation.z = Math.PI / 2;  // Rotate from +Y to -X (left)
    this.deviceGroup.add(xAxis);
    
    // Device Y axis (green) - maps to Three.js +Y (up)
    const yAxis = this.createArrow(COLORS.AXIS_Y, axisLength);
    // No rotation - default arrow points +Y which is up
    this.deviceGroup.add(yAxis);
    
    // Device Z axis (blue) - maps to Three.js -Z (forward, into screen)
    const zAxis = this.createArrow(COLORS.AXIS_Z, axisLength);
    zAxis.rotation.x = -Math.PI / 2;  // Rotate from +Y to -Z (forward into screen)
    this.deviceGroup.add(zAxis);
    
    // Axis labels positioned in Three.js local frame
    this.addAxisLabel('X', new THREE.Vector3(-(axisLength + 0.15), 0, 0), COLORS.AXIS_X);
    this.addAxisLabel('Y', new THREE.Vector3(0, axisLength + 0.15, 0), COLORS.AXIS_Y);
    this.addAxisLabel('Z', new THREE.Vector3(0, 0, -(axisLength + 0.15)), COLORS.AXIS_Z);
  }
  
  /**
   * Create an arrow for axis visualization
   */
  private createArrow(color: number, length: number): THREE.Group {
    const arrow = new THREE.Group();
    
    // Shaft
    const shaftGeometry = new THREE.CylinderGeometry(0.02, 0.02, length - 0.15, 8);
    const shaftMaterial = new THREE.MeshPhongMaterial({ color });
    const shaft = new THREE.Mesh(shaftGeometry, shaftMaterial);
    shaft.position.y = length / 2 - 0.075;
    arrow.add(shaft);
    
    // Cone
    const coneGeometry = new THREE.ConeGeometry(0.06, 0.15, 8);
    const coneMaterial = new THREE.MeshPhongMaterial({ color });
    const cone = new THREE.Mesh(coneGeometry, coneMaterial);
    cone.position.y = length - 0.075;
    arrow.add(cone);
    
    return arrow;
  }
  
  /**
   * Add a text label for axis
   */
  private addAxisLabel(text: string, position: THREE.Vector3, color: number): void {
    const canvas = document.createElement('canvas');
    const context = canvas.getContext('2d')!;
    canvas.width = 64;
    canvas.height = 64;
    
    context.fillStyle = '#' + color.toString(16).padStart(6, '0');
    context.font = 'bold 48px Arial';
    context.textAlign = 'center';
    context.textBaseline = 'middle';
    context.fillText(text, 32, 32);
    
    const texture = new THREE.CanvasTexture(canvas);
    const spriteMaterial = new THREE.SpriteMaterial({ map: texture });
    const sprite = new THREE.Sprite(spriteMaterial);
    sprite.position.copy(position);
    sprite.scale.set(0.3, 0.3, 1);
    this.deviceGroup.add(sprite);
  }
  
  /**
   * Create the world reference frame
   * 
   * World frame convention (viewing from above, Y pointing up out of screen):
   *   North = into screen (-Z in Three.js)
   *   East = right (+X in Three.js)
   *   Up = up (+Y in Three.js)
   */
  private createReferenceFrame(): void {
    const refGroup = new THREE.Group();
    const axisLength = 2.0;
    const opacity = 0.3;
    
    // North - into screen (-Z), pink dashed
    const northGeometry = new THREE.BufferGeometry().setFromPoints([
      new THREE.Vector3(0, 0, 0),
      new THREE.Vector3(0, 0, -axisLength)
    ]);
    const northMaterial = new THREE.LineDashedMaterial({ 
      color: 0xff88aa, 
      dashSize: 0.1, 
      gapSize: 0.05,
      opacity,
      transparent: true
    });
    const north = new THREE.Line(northGeometry, northMaterial);
    north.computeLineDistances();
    refGroup.add(north);
    
    // East - right (+X), green dashed
    const eastGeometry = new THREE.BufferGeometry().setFromPoints([
      new THREE.Vector3(0, 0, 0),
      new THREE.Vector3(axisLength, 0, 0)
    ]);
    const eastMaterial = new THREE.LineDashedMaterial({ 
      color: 0x44ff44, 
      dashSize: 0.1, 
      gapSize: 0.05,
      opacity,
      transparent: true
    });
    const east = new THREE.Line(eastGeometry, eastMaterial);
    east.computeLineDistances();
    refGroup.add(east);
    
    // Down - down (-Y), blue dashed
    const downGeometry = new THREE.BufferGeometry().setFromPoints([
      new THREE.Vector3(0, 0, 0),
      new THREE.Vector3(0, -axisLength, 0)
    ]);
    const downMaterial = new THREE.LineDashedMaterial({ 
      color: 0x4444ff, 
      dashSize: 0.1, 
      gapSize: 0.05,
      opacity,
      transparent: true
    });
    const down = new THREE.Line(downGeometry, downMaterial);
    down.computeLineDistances();
    refGroup.add(down);
    
    // North label (at -Z, into screen)
    this.addWorldLabel('N', new THREE.Vector3(0, 0, -(axisLength + 0.2)));
    
    this.scene.add(refGroup);
  }
  
  /**
   * Add world frame label
   */
  private addWorldLabel(text: string, position: THREE.Vector3): void {
    const canvas = document.createElement('canvas');
    const context = canvas.getContext('2d')!;
    canvas.width = 64;
    canvas.height = 64;
    
    context.fillStyle = '#ffffff';
    context.font = 'bold 48px Arial';
    context.textAlign = 'center';
    context.textBaseline = 'middle';
    context.fillText(text, 32, 32);
    
    const texture = new THREE.CanvasTexture(canvas);
    const spriteMaterial = new THREE.SpriteMaterial({ 
      map: texture,
      opacity: 0.5,
      transparent: true
    });
    const sprite = new THREE.Sprite(spriteMaterial);
    sprite.position.copy(position);
    sprite.scale.set(0.4, 0.4, 1);
    this.scene.add(sprite);
  }
  
  /**
   * Create lighting
   */
  private createLighting(): void {
    // Ambient light
    const ambient = new THREE.AmbientLight(0x404040, 0.5);
    this.scene.add(ambient);
    
    // Main directional light
    const mainLight = new THREE.DirectionalLight(0xffffff, 1);
    mainLight.position.set(5, 5, 5);
    this.scene.add(mainLight);
    
    // Fill light
    const fillLight = new THREE.DirectionalLight(0x4488ff, 0.3);
    fillLight.position.set(-5, 0, -5);
    this.scene.add(fillLight);
  }
  
  /**
   * Create ground grid
   */
  private createGrid(): void {
    const gridHelper = new THREE.GridHelper(4, 8, 0x444444, 0x222222);
    gridHelper.position.y = -1.5;
    this.scene.add(gridHelper);
  }
  
  /**
   * Update device orientation from quaternion
   * 
   * The AHRS outputs orientation in NWU frame (X=North, Y=West, Z=Up).
   * The Three.js model is set up to match the body frame visualization.
   * 
   * Rather than transform the quaternion components (which is complex),
   * we apply a coordinate system rotation.
   * 
   * NWU world frame: X=North, Y=West, Z=Up
   * Three.js world: X=East, Y=Up, Z=South (looking at -Z for North)
   * 
   * To convert: we need to rotate the NWU frame to align with Three.js
   * - NWU_X (North) should become Three.js -Z
   * - NWU_Y (West) should become Three.js -X
   * - NWU_Z (Up) should become Three.js +Y
   */
  setOrientation(q: Quaternion): void {
    // Transform quaternion from NWU frame to Three.js frame
    //
    // Quaternion component mapping for coordinate frame change:
    // In NWU: qx = rotation around North, qy = rotation around West, qz = rotation around Up
    // In Three.js: X = East, Y = Up, Z = South
    //
    // Mapping:
    //   NWU qx (North rot) -> Three.js -qz (North = -Z)
    //   NWU qy (West rot) -> Three.js -qx (West = -X)
    //   NWU qz (Up rot) -> Three.js qy (Up = Y)
    
    this.deviceGroup.quaternion.set(-q.y, q.z, -q.x, q.w);
  }
  
  /**
   * Reset to identity orientation
   */
  reset(): void {
    this.deviceGroup.quaternion.set(0, 0, 0, 1);
  }
  
  /**
   * Handle window resize
   */
  private onResize(): void {
    const width = this.container.clientWidth;
    const height = this.container.clientHeight;
    
    this.camera.aspect = width / height;
    this.camera.updateProjectionMatrix();
    this.renderer.setSize(width, height);
  }
  
  /**
   * Animation loop
   */
  private animate(): void {
    this.animationId = requestAnimationFrame(() => this.animate());
    
    // Update orbit controls (required for damping)
    this.controls.update();
    
    // Auto-rotate when showing mag or IMU plot
    if (this.showingMagPlot || this.showingIMUPlot) {
      this.controls.autoRotate = true;
      this.controls.autoRotateSpeed = 2.0;
    } else {
      this.controls.autoRotate = false;
    }
    
    this.renderer.render(this.scene, this.camera);
  }
  
  /**
   * Toggle magnetometer 3D scatter plot
   * Shows raw mag data for calibration visualization
   */
  toggleMagPlot(
    samples: MAGData[],
    calibration: { offsetX: number; offsetY: number; offsetZ: number }
  ): void {
    // If already showing, remove it
    if (this.magPlotGroup) {
      this.scene.remove(this.magPlotGroup);
      this.magPlotGroup = null;
      this.showingMagPlot = false;
      this.deviceGroup.visible = true;
      
      // Reset camera
      this.camera.position.set(3, 2, 3);
      this.camera.lookAt(0, 0, 0);
      return;
    }
    
    // Create new plot group
    this.magPlotGroup = new THREE.Group();
    this.showingMagPlot = true;
    this.deviceGroup.visible = false;
    
    // Scale factor to make the plot visible (mag values are small)
    const scale = 2.0;
    
    // Create points for raw data (red)
    const rawGeometry = new THREE.BufferGeometry();
    const rawPositions: number[] = [];
    
    // Create points for calibrated data (green)
    const calGeometry = new THREE.BufferGeometry();
    const calPositions: number[] = [];
    
    for (const sample of samples) {
      // Use raw sensor data (no transform - axis remap is separate)
      const x = sample.x;
      const y = sample.y;
      const z = sample.z;
      
      // Raw positions (before calibration)
      rawPositions.push(x * scale, y * scale, z * scale);
      
      // Calibrated positions
      const cx = (x - calibration.offsetX) * scale;
      const cy = (y - calibration.offsetY) * scale;
      const cz = (z - calibration.offsetZ) * scale;
      calPositions.push(cx, cy, cz);
    }
    
    rawGeometry.setAttribute('position', new THREE.Float32BufferAttribute(rawPositions, 3));
    calGeometry.setAttribute('position', new THREE.Float32BufferAttribute(calPositions, 3));
    
    // Raw points (red, semi-transparent)
    const rawMaterial = new THREE.PointsMaterial({
      color: 0xff4444,
      size: 0.03,
      opacity: 0.5,
      transparent: true
    });
    const rawPoints = new THREE.Points(rawGeometry, rawMaterial);
    this.magPlotGroup.add(rawPoints);
    
    // Calibrated points (green)
    const calMaterial = new THREE.PointsMaterial({
      color: 0x44ff44,
      size: 0.04,
      opacity: 0.8,
      transparent: true
    });
    const calPoints = new THREE.Points(calGeometry, calMaterial);
    this.magPlotGroup.add(calPoints);
    
    // Add center sphere (where calibrated data should be centered)
    const centerGeometry = new THREE.SphereGeometry(0.05, 16, 16);
    const centerMaterial = new THREE.MeshBasicMaterial({ color: 0xffff00 });
    const center = new THREE.Mesh(centerGeometry, centerMaterial);
    this.magPlotGroup.add(center);
    
    // Add reference sphere showing expected field magnitude (~0.5 gauss)
    const expectedMag = 0.5 * scale;
    const sphereGeometry = new THREE.SphereGeometry(expectedMag, 32, 32);
    const sphereMaterial = new THREE.MeshBasicMaterial({
      color: 0x4488ff,
      wireframe: true,
      opacity: 0.3,
      transparent: true
    });
    const sphere = new THREE.Mesh(sphereGeometry, sphereMaterial);
    this.magPlotGroup.add(sphere);
    
    // Add axes
    const axesHelper = new THREE.AxesHelper(1.5);
    this.magPlotGroup.add(axesHelper);
    
    // Add calibration center marker
    const calCenterGeometry = new THREE.SphereGeometry(0.08, 16, 16);
    const calCenterMaterial = new THREE.MeshBasicMaterial({ 
      color: 0xff00ff,
      opacity: 0.7,
      transparent: true
    });
    const calCenter = new THREE.Mesh(calCenterGeometry, calCenterMaterial);
    calCenter.position.set(
      calibration.offsetX * scale,
      calibration.offsetY * scale,
      calibration.offsetZ * scale
    );
    this.magPlotGroup.add(calCenter);
    
    this.scene.add(this.magPlotGroup);
    
    // Adjust camera for better view
    this.camera.position.set(3, 3, 3);
    this.camera.lookAt(0, 0, 0);
  }
  
  /**
   * Toggle IMU accelerometer 3D plot visualization
   * Shows raw accel data for calibration visualization
   */
  toggleIMUPlot(
    samples: { ax: number; ay: number; az: number }[],
    calibration: { offsetX: number; offsetY: number; offsetZ: number }
  ): void {
    // If already showing, remove it
    if (this.imuPlotGroup) {
      this.scene.remove(this.imuPlotGroup);
      this.imuPlotGroup = null;
      this.showingIMUPlot = false;
      this.deviceGroup.visible = true;
      
      // Reset camera
      this.camera.position.set(3, 2, 3);
      this.camera.lookAt(0, 0, 0);
      return;
    }
    
    // Create new plot group
    this.imuPlotGroup = new THREE.Group();
    this.showingIMUPlot = true;
    this.deviceGroup.visible = false;
    
    // Scale factor (accel values are typically around 1g)
    const scale = 1.5;
    
    // Create points for raw data (red)
    const rawGeometry = new THREE.BufferGeometry();
    const rawPositions: number[] = [];
    
    // Create points for calibrated data (green)
    const calGeometry = new THREE.BufferGeometry();
    const calPositions: number[] = [];
    
    for (const sample of samples) {
      // Use raw accelerometer data
      const x = sample.ax;
      const y = sample.ay;
      const z = sample.az;
      
      // Raw positions (before calibration)
      rawPositions.push(x * scale, y * scale, z * scale);
      
      // Calibrated positions
      const cx = (x - calibration.offsetX) * scale;
      const cy = (y - calibration.offsetY) * scale;
      const cz = (z - calibration.offsetZ) * scale;
      calPositions.push(cx, cy, cz);
    }
    
    rawGeometry.setAttribute('position', new THREE.Float32BufferAttribute(rawPositions, 3));
    calGeometry.setAttribute('position', new THREE.Float32BufferAttribute(calPositions, 3));
    
    // Raw points (red, semi-transparent)
    const rawMaterial = new THREE.PointsMaterial({
      color: 0xff4444,
      size: 0.02,
      opacity: 0.5,
      transparent: true
    });
    const rawPoints = new THREE.Points(rawGeometry, rawMaterial);
    this.imuPlotGroup.add(rawPoints);
    
    // Calibrated points (green)
    const calMaterial = new THREE.PointsMaterial({
      color: 0x44ff44,
      size: 0.025,
      opacity: 0.8,
      transparent: true
    });
    const calPoints = new THREE.Points(calGeometry, calMaterial);
    this.imuPlotGroup.add(calPoints);
    
    // Add center sphere (origin)
    const centerGeometry = new THREE.SphereGeometry(0.03, 16, 16);
    const centerMaterial = new THREE.MeshBasicMaterial({ color: 0xffff00 });
    const center = new THREE.Mesh(centerGeometry, centerMaterial);
    this.imuPlotGroup.add(center);
    
    // Add reference sphere showing expected 1g magnitude
    const expectedMag = 1.0 * scale;
    const sphereGeometry = new THREE.SphereGeometry(expectedMag, 32, 32);
    const sphereMaterial = new THREE.MeshBasicMaterial({
      color: 0x4488ff,
      wireframe: true,
      opacity: 0.3,
      transparent: true
    });
    const sphere = new THREE.Mesh(sphereGeometry, sphereMaterial);
    this.imuPlotGroup.add(sphere);
    
    // Add axes
    const axesHelper = new THREE.AxesHelper(2.0);
    this.imuPlotGroup.add(axesHelper);
    
    // Add calibration center marker (where the raw data is centered)
    const calCenterGeometry = new THREE.SphereGeometry(0.05, 16, 16);
    const calCenterMaterial = new THREE.MeshBasicMaterial({ 
      color: 0xff00ff,
      opacity: 0.7,
      transparent: true
    });
    const calCenter = new THREE.Mesh(calCenterGeometry, calCenterMaterial);
    calCenter.position.set(
      calibration.offsetX * scale,
      calibration.offsetY * scale,
      calibration.offsetZ * scale
    );
    this.imuPlotGroup.add(calCenter);
    
    this.scene.add(this.imuPlotGroup);
    
    // Adjust camera for better view
    this.camera.position.set(3, 3, 3);
    this.camera.lookAt(0, 0, 0);
  }
  
  /**
   * Check if IMU plot is currently visible
   */
  isIMUPlotVisible(): boolean {
    return this.showingIMUPlot;
  }
  
  /**
   * Check if Mag plot is currently visible
   */
  isMagPlotVisible(): boolean {
    return this.showingMagPlot;
  }
  
  /**
   * Refresh IMU plot with new calibration (if plot is visible)
   * Call this after changing calibration values
   */
  refreshIMUPlot(
    samples: { ax: number; ay: number; az: number }[],
    calibration: { offsetX: number; offsetY: number; offsetZ: number }
  ): void {
    if (!this.showingIMUPlot) return;
    
    // Remove current plot
    if (this.imuPlotGroup) {
      this.scene.remove(this.imuPlotGroup);
      this.imuPlotGroup = null;
    }
    this.showingIMUPlot = false;
    
    // Re-create with new calibration (toggleIMUPlot will create since showingIMUPlot is false)
    this.toggleIMUPlot(samples, calibration);
  }
  
  /**
   * Refresh Mag plot with new calibration (if plot is visible)
   * Call this after changing calibration values
   */
  refreshMagPlot(
    samples: MAGData[],
    calibration: { offsetX: number; offsetY: number; offsetZ: number }
  ): void {
    if (!this.showingMagPlot) return;
    
    // Remove current plot
    if (this.magPlotGroup) {
      this.scene.remove(this.magPlotGroup);
      this.magPlotGroup = null;
    }
    this.showingMagPlot = false;
    
    // Re-create with new calibration
    this.toggleMagPlot(samples, calibration);
  }
  
  /**
   * Toggle sensor vector visualization
   */
  toggleSensorVectors(show: boolean): void {
    this.showSensorVectors = show;
    this.sensorVectorsGroup.visible = show;
    if (this.magWorldArrow) {
      this.magWorldArrow.visible = show;
    }
  }
  
  /**
   * Toggle linear acceleration vector (body frame, gravity removed)
   */
  toggleLinearAccel(show: boolean): void {
    this.showLinearAccel = show;
    if (this.linearAccelArrow) {
      this.linearAccelArrow.visible = show;
    }
    if (this.linearAccelLabel) {
      this.linearAccelLabel.visible = show;
    }
  }
  
  /**
   * Toggle earth acceleration vector (world frame, gravity removed)
   */
  toggleEarthAccel(show: boolean): void {
    this.showEarthAccel = show;
    if (this.earthAccelArrow) {
      this.earthAccelArrow.visible = show;
    }
    if (this.earthAccelLabel) {
      this.earthAccelLabel.visible = show;
    }
  }
  
  /**
   * Toggle AHRS gravity vector (body frame, estimated from orientation)
   */
  toggleGravity(show: boolean): void {
    this.showGravity = show;
    if (this.gravityArrow) {
      this.gravityArrow.visible = show;
    }
    if (this.gravityLabel) {
      this.gravityLabel.visible = show;
    }
  }
  
  /**
   * Toggle heading vector (compass direction in world frame)
   */
  toggleHeading(show: boolean): void {
    this.showHeading = show;
    if (this.headingArrow) {
      this.headingArrow.visible = show;
    }
    if (this.headingLabel) {
      this.headingLabel.visible = show;
    }
  }
  
  /**
   * Update heading vector display
   * Shows the magnetic heading - the horizontal component of the magnetometer
   * in world frame, pointing toward magnetic north.
   * 
   * This is computed by taking the calibrated mag reading, transforming it
   * to world frame via the device quaternion, and projecting onto horizontal.
   * 
   * @param _headingDeg Heading in degrees (not used - we derive from mag directly)
   */
  updateHeadingVector(_headingDeg: number): void {
    // Remove old heading arrow
    if (this.headingArrow) {
      this.scene.remove(this.headingArrow);
      this.headingArrow.dispose();
      this.headingArrow = null;
    }
    if (this.headingLabel) {
      this.scene.remove(this.headingLabel);
      this.disposeLabel(this.headingLabel);
      this.headingLabel = null;
    }
    
    if (!this.showHeading) return;
    
    // If we have the mag world direction stored, use it projected to horizontal
    // This shows where the magnetometer says north is
    if (this.magWorldDirection) {
      // Project onto horizontal plane (zero out Y component)
      const headingDir = new THREE.Vector3(
        this.magWorldDirection.x,
        0,
        this.magWorldDirection.z
      );
      
      // If the projection is too small (mag pointing straight up/down), skip
      if (headingDir.length() < 0.1) {
        return;
      }
      
      headingDir.normalize();
      
      const arrowLength = 2.0;
      
      // Create heading arrow (green color for visibility)
      this.headingArrow = new THREE.ArrowHelper(
        headingDir,
        new THREE.Vector3(0, 0, 0),
        arrowLength,
        0x00ff00,  // Green
        0.2,
        0.1
      );
      this.scene.add(this.headingArrow);
      
      // Add "H" label at tip
      this.headingLabel = this.createTextLabel('H', '#00ff00');
      this.headingLabel.position.copy(headingDir.clone().multiplyScalar(arrowLength + 0.15));
      this.scene.add(this.headingLabel);
    }
  }
  
  /**
   * Update sensor vectors (raw readings in body frame)
   * These show the actual sensor readings attached to the device
   * Also shows mag in WORLD frame (cyan) so it stays fixed even if AHRS drifts
   * 
   * @param accel Accelerometer reading [x, y, z] in g
   * @param mag Magnetometer reading [x, y, z] in gauss (after calibration and remap)
   * @param gyro Gyroscope reading [x, y, z] in rad/s (after calibration and remap)
   */
  updateSensorVectors(
    accel: { x: number; y: number; z: number },
    mag: { x: number; y: number; z: number } | null,
    gyro: { x: number; y: number; z: number } | null = null
  ): void {
    if (!this.showSensorVectors) return;
    
    // Remove old arrows
    if (this.accelArrow) {
      this.sensorVectorsGroup.remove(this.accelArrow);
      this.accelArrow.dispose();
    }
    if (this.accelLabel) {
      this.sensorVectorsGroup.remove(this.accelLabel);
      this.disposeLabel(this.accelLabel);
      this.accelLabel = null;
    }
    if (this.magArrow) {
      this.sensorVectorsGroup.remove(this.magArrow);
      this.magArrow.dispose();
    }
    if (this.magLabel) {
      this.sensorVectorsGroup.remove(this.magLabel);
      this.disposeLabel(this.magLabel);
      this.magLabel = null;
    }
    if (this.magWorldArrow) {
      this.scene.remove(this.magWorldArrow);
      this.magWorldArrow.dispose();
      this.magWorldArrow = null;
    }
    if (this.magWorldLabel) {
      this.scene.remove(this.magWorldLabel);
      this.disposeLabel(this.magWorldLabel);
      this.magWorldLabel = null;
    }
    
    // Remove old gyro arrows
    if (this.gyroArrowX) {
      this.sensorVectorsGroup.remove(this.gyroArrowX);
      disposeCurvedArrow(this.gyroArrowX);
      this.gyroArrowX = null;
    }
    if (this.gyroArrowY) {
      this.sensorVectorsGroup.remove(this.gyroArrowY);
      disposeCurvedArrow(this.gyroArrowY);
      this.gyroArrowY = null;
    }
    if (this.gyroArrowZ) {
      this.sensorVectorsGroup.remove(this.gyroArrowZ);
      disposeCurvedArrow(this.gyroArrowZ);
      this.gyroArrowZ = null;
    }
    
    // Sensor readings are in device body frame:
    //   Device X = left (West when facing North), Device Y = up, Device Z = forward (out front face)
    // Three.js local frame:
    //   Three.js -X = left (West), Three.js Y = up, Three.js -Z = forward (North)
    // So: Device X → -X, Device Y → +Y, Device Z → -Z
    const sensorToLocal = (v: { x: number; y: number; z: number }) => {
      return new THREE.Vector3(-v.x, v.y, -v.z);
    };
    
    // Accelerometer vector (yellow) - shows gravity direction in body frame
    const accelLocal = sensorToLocal(accel);
    const accelDir = accelLocal.clone().normalize();
    const accelLength = accelLocal.length();
    const accelArrowLength = accelLength * 1.5;
    this.accelArrow = new THREE.ArrowHelper(
      accelDir,
      new THREE.Vector3(0, 0, 0),
      accelArrowLength,  // Scale for visibility
      0xffff00,  // Yellow
      0.2,
      0.1
    );
    this.sensorVectorsGroup.add(this.accelArrow);
    
    // Add "G" label at tip of gravity vector
    this.accelLabel = this.createTextLabel('G', '#ffff00');
    this.accelLabel.position.copy(accelDir.clone().multiplyScalar(accelArrowLength + 0.15));
    this.sensorVectorsGroup.add(this.accelLabel);
    
    // Magnetometer vector (magenta) - shows magnetic north in body frame (attached to device)
    if (mag) {
      const magLocal = sensorToLocal(mag);
      const magDir = magLocal.clone().normalize();
      const magLength = magLocal.length();
      const magArrowLength = magLength * 3.0;
      this.magArrow = new THREE.ArrowHelper(
        magDir,
        new THREE.Vector3(0, 0, 0),
        magArrowLength,  // Scale for visibility (mag values are smaller)
        0xff00ff,  // Magenta - body frame
        0.2,
        0.1
      );
      this.sensorVectorsGroup.add(this.magArrow);
      
      // Add "M" label at tip of mag vector (body frame)
      this.magLabel = this.createTextLabel('M', '#ff00ff');
      this.magLabel.position.copy(magDir.clone().multiplyScalar(magArrowLength + 0.15));
      this.sensorVectorsGroup.add(this.magLabel);
      
      // Magnetometer vector in WORLD frame (cyan)
      // Transform body-frame mag to world frame using current device orientation
      // This should stay pointing at magnetic north regardless of AHRS drift
      const magLocalVec = magLocal.clone();
      magLocalVec.applyQuaternion(this.deviceGroup.quaternion);
      const magWorldDir = magLocalVec.normalize();
      
      // Store for heading vector use
      this.magWorldDirection = magWorldDir.clone();
      
      const magWorldArrowLength = magLength * 3.0;
      this.magWorldArrow = new THREE.ArrowHelper(
        magWorldDir,
        new THREE.Vector3(0, 0, 0),
        magWorldArrowLength,
        0x00ffff,  // Cyan - world frame
        0.2,
        0.1
      );
      this.scene.add(this.magWorldArrow);
      
      // Add "M" label at tip of mag vector (world frame) - smaller to differentiate
      this.magWorldLabel = this.createTextLabel('M', '#00ffff');
      this.magWorldLabel.position.copy(magWorldDir.clone().multiplyScalar(magWorldArrowLength + 0.15));
      this.scene.add(this.magWorldLabel);
    }
    
    // Gyroscope curved arrows - show angular velocity around each axis
    // Gyro values are in rad/s, we scale by 0.5 for visibility
    if (gyro) {
      const gyroScale = 0.5;
      
      // Transform gyro to local Three.js frame: Device X → -X, Device Y → +Y, Device Z → -Z
      const gxLocal = -gyro.x * gyroScale;
      const gyLocal = gyro.y * gyroScale;
      const gzLocal = -gyro.z * gyroScale;
      
      // X rotation (light red) - rotation around X axis
      if (Math.abs(gxLocal) > 0.01) {
        this.gyroArrowX = createCurvedArrow('x', gxLocal, COLORS.GYRO_X, { radius: 0.9 });
        this.sensorVectorsGroup.add(this.gyroArrowX);
      }
      
      // Y rotation (light green) - rotation around Y axis
      if (Math.abs(gyLocal) > 0.01) {
        this.gyroArrowY = createCurvedArrow('y', gyLocal, COLORS.GYRO_Y, { radius: 0.9 });
        this.sensorVectorsGroup.add(this.gyroArrowY);
      }
      
      // Z rotation (light blue) - rotation around Z axis
      if (Math.abs(gzLocal) > 0.01) {
        this.gyroArrowZ = createCurvedArrow('z', gzLocal, COLORS.GYRO_Z, { radius: 0.9 });
        this.sensorVectorsGroup.add(this.gyroArrowZ);
      }
    }
  }
  
  /**
   * Update linear acceleration, earth acceleration, and gravity vectors
   * 
   * @param linearAccel Linear acceleration in body frame (gravity removed), in g
   * @param earthAccel Earth-frame acceleration in NWU (gravity removed), in g
   * @param gravity AHRS-estimated gravity direction in body frame (unit vector)
   */
  updateAccelerationVectors(
    linearAccel: { x: number; y: number; z: number } | null,
    earthAccel: { x: number; y: number; z: number } | null,
    gravity?: { x: number; y: number; z: number } | null
  ): void {
    // Clean up old arrows
    if (this.linearAccelArrow) {
      this.sensorVectorsGroup.remove(this.linearAccelArrow);
      this.linearAccelArrow.dispose();
      this.linearAccelArrow = null;
    }
    if (this.linearAccelLabel) {
      this.sensorVectorsGroup.remove(this.linearAccelLabel);
      this.disposeLabel(this.linearAccelLabel);
      this.linearAccelLabel = null;
    }
    if (this.earthAccelArrow) {
      this.scene.remove(this.earthAccelArrow);
      this.earthAccelArrow.dispose();
      this.earthAccelArrow = null;
    }
    if (this.earthAccelLabel) {
      this.scene.remove(this.earthAccelLabel);
      this.disposeLabel(this.earthAccelLabel);
      this.earthAccelLabel = null;
    }
    if (this.gravityArrow) {
      this.scene.remove(this.gravityArrow);  // Remove from scene, not sensorVectorsGroup
      this.gravityArrow.dispose();
      this.gravityArrow = null;
    }
    if (this.gravityLabel) {
      this.scene.remove(this.gravityLabel);  // Remove from scene, not sensorVectorsGroup
      this.disposeLabel(this.gravityLabel);
      this.gravityLabel = null;
    }
    
    // =========================================================================
    // Coordinate Transforms for Viewer
    // =========================================================================
    
    // Body frame → Three.js local (attached to device)
    // Body: X=West, Y=Up, Z=North
    // Three.js local: X=right(-West), Y=up, Z=back(-North when at identity)
    // Transform: Three.js = [-Body_X, +Body_Y, -Body_Z]
    const bodyToThreeLocal = (v: { x: number; y: number; z: number }) => {
      return new THREE.Vector3(-v.x, v.y, -v.z);
    };
    
    // NWU world frame → Three.js world
    // NWU: X=North, Y=West, Z=Up
    // Three.js world: X=East, Y=Up, Z=South (camera looks at -Z for North)
    // Transform: Three.js = [-NWU_Y, +NWU_Z, -NWU_X]
    const nwuToThreeWorld = (v: { x: number; y: number; z: number }) => {
      return new THREE.Vector3(-v.y, v.z, -v.x);
    };
    
    // =========================================================================
    // Linear Acceleration (body frame, gravity removed)
    // =========================================================================
    // Attached to device, shows acceleration relative to the device's orientation
    if (linearAccel && this.showLinearAccel) {
      const linearLocal = bodyToThreeLocal(linearAccel);
      const linearMag = linearLocal.length();
      
      // Only show if significant acceleration (> 0.02g threshold)
      if (linearMag > 0.02) {
        const linearDir = linearLocal.clone().normalize();
        const linearArrowLength = Math.min(linearMag * 3.0, 2.0);  // Scale and cap length
        
        this.linearAccelArrow = new THREE.ArrowHelper(
          linearDir,
          new THREE.Vector3(0, 0, 0),
          linearArrowLength,
          0xff69b4,  // Pink (Hot Pink)
          0.15,
          0.08
        );
        this.sensorVectorsGroup.add(this.linearAccelArrow);
        
        // Add "L" label
        this.linearAccelLabel = this.createTextLabel('L', '#ff69b4');
        this.linearAccelLabel.position.copy(linearDir.clone().multiplyScalar(linearArrowLength + 0.15));
        this.sensorVectorsGroup.add(this.linearAccelLabel);
      }
    }
    
    // =========================================================================
    // Earth Acceleration (NWU world frame, gravity removed)
    // =========================================================================
    // Fixed in world space, shows actual motion direction regardless of device orientation
    if (earthAccel && this.showEarthAccel) {
      const earthWorld = nwuToThreeWorld(earthAccel);
      const earthMag = earthWorld.length();
      
      // Only show if significant acceleration (> 0.02g threshold)
      if (earthMag > 0.02) {
        const earthDir = earthWorld.clone().normalize();
        const earthArrowLength = Math.min(earthMag * 3.0, 2.0);  // Scale and cap length
        
        this.earthAccelArrow = new THREE.ArrowHelper(
          earthDir,
          new THREE.Vector3(0, 0, 0),
          earthArrowLength,
          0xffa500,  // Orange
          0.15,
          0.08
        );
        this.scene.add(this.earthAccelArrow);  // Add to scene, not device group
        
        // Add "E" label
        this.earthAccelLabel = this.createTextLabel('E', '#ffa500');
        this.earthAccelLabel.position.copy(earthDir.clone().multiplyScalar(earthArrowLength + 0.15));
        this.scene.add(this.earthAccelLabel);
      }
    }
    
    // =========================================================================
    // AHRS Gravity Vector (world frame - always points up)
    // =========================================================================
    // Shows the AHRS estimate of gravity direction in world frame.
    // This is always "up" in world coordinates (the direction opposite to gravity).
    // When stationary and level, G and Gv both point up in world.
    // When device tilts, G (raw accel) tilts with device, but Gv stays pointing up.
    // 
    // The gravity vector IS the world up direction. We display it directly in
    // world frame (scene), not in the rotating sensorVectorsGroup.
    if (gravity && this.showGravity) {
      // Gravity from AHRS is in body frame - but we want world frame.
      // In world frame, gravity reaction always points UP = [0, 0, +1] in NWU.
      // Convert NWU world up to Three.js world coordinates.
      const gravWorld = nwuToThreeWorld({ x: 0, y: 0, z: 1 });  // NWU up = [0,0,1]
      const gravDir = gravWorld.normalize();
      const gravArrowLength = 1.5;
      
      this.gravityArrow = new THREE.ArrowHelper(
        gravDir,
        new THREE.Vector3(0, 0, 0),
        gravArrowLength,
        0x00ffff,  // Cyan - distinct from yellow raw accel
        0.15,
        0.08
      );
      this.scene.add(this.gravityArrow);  // Add to scene, NOT sensorVectorsGroup
      
      // Add "Gv" label (Gravity Vector from AHRS)
      this.gravityLabel = this.createTextLabel('Gv', '#00ffff');
      this.gravityLabel.position.copy(gravDir.clone().multiplyScalar(gravArrowLength + 0.15));
      this.scene.add(this.gravityLabel);  // Add to scene, NOT sensorVectorsGroup
    }
  }
  
  /**
   * Clean up resources
   */
  dispose(): void {
    if (this.animationId !== null) {
      cancelAnimationFrame(this.animationId);
    }
    this.renderer.dispose();
    this.container.removeChild(this.renderer.domElement);
  }
  
  /**
   * Create a text sprite label that always faces the camera
   */
  private createTextLabel(text: string, color: string): THREE.Sprite {
    const canvas = document.createElement('canvas');
    const size = 64;
    canvas.width = size;
    canvas.height = size;
    
    const context = canvas.getContext('2d')!;
    context.fillStyle = color;
    context.font = 'bold 48px Arial';
    context.textAlign = 'center';
    context.textBaseline = 'middle';
    context.fillText(text, size / 2, size / 2);
    
    const texture = new THREE.CanvasTexture(canvas);
    const material = new THREE.SpriteMaterial({ 
      map: texture,
      transparent: true,
      depthTest: false  // Always render on top
    });
    const sprite = new THREE.Sprite(material);
    sprite.scale.set(0.3, 0.3, 1);  // Adjust size
    
    return sprite;
  }
  
  /**
   * Dispose of a text label sprite
   */
  private disposeLabel(label: THREE.Sprite | null): void {
    if (label) {
      label.material.map?.dispose();
      label.material.dispose();
    }
  }
}
