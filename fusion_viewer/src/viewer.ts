/**
 * Three.js 3D Viewer for Sensor Fusion Orientation
 * 
 * Displays a 3D model representing the FlySight device orientation
 * with reference frame indicators.
 */

import * as THREE from 'three';
import type { Quaternion } from './fusion';
import type { MAGData } from './csvParser';

export class OrientationViewer {
  private container: HTMLElement;
  private scene: THREE.Scene;
  private camera: THREE.PerspectiveCamera;
  private renderer: THREE.WebGLRenderer;
  private deviceGroup: THREE.Group;
  private magPlotGroup: THREE.Group | null = null;
  private animationId: number | null = null;
  private showingMagPlot: boolean = false;
  
  constructor(containerId: string) {
    const container = document.getElementById(containerId);
    if (!container) {
      throw new Error(`Container element '${containerId}' not found`);
    }
    this.container = container;
    
    // Initialize Three.js
    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color(0x1a1a2e);
    
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
    
    // Device group (will be rotated)
    this.deviceGroup = new THREE.Group();
    this.scene.add(this.deviceGroup);
    
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
   */
  private createDevice(): void {
    // Main body - rectangular prism representing FlySight
    const bodyGeometry = new THREE.BoxGeometry(0.6, 1.2, 0.3);
    const bodyMaterial = new THREE.MeshPhongMaterial({ 
      color: 0x2a2a4a,
      specular: 0x444444,
      shininess: 30
    });
    const body = new THREE.Mesh(bodyGeometry, bodyMaterial);
    this.deviceGroup.add(body);
    
    // LED indicator (top front)
    const ledGeometry = new THREE.SphereGeometry(0.05, 16, 16);
    const ledMaterial = new THREE.MeshBasicMaterial({ color: 0x00ff00 });
    const led = new THREE.Mesh(ledGeometry, ledMaterial);
    led.position.set(0, 0.55, 0.16);
    this.deviceGroup.add(led);
    
    // USB port indicator (bottom)
    const usbGeometry = new THREE.BoxGeometry(0.15, 0.05, 0.1);
    const usbMaterial = new THREE.MeshPhongMaterial({ color: 0x444444 });
    const usb = new THREE.Mesh(usbGeometry, usbMaterial);
    usb.position.set(0, -0.6, 0.1);
    this.deviceGroup.add(usb);
    
    // Front face indicator (screen area)
    const screenGeometry = new THREE.PlaneGeometry(0.4, 0.6);
    const screenMaterial = new THREE.MeshPhongMaterial({ 
      color: 0x111111,
      side: THREE.FrontSide
    });
    const screen = new THREE.Mesh(screenGeometry, screenMaterial);
    screen.position.set(0, 0, 0.151);
    this.deviceGroup.add(screen);
    
    // Device axes (body frame)
    const axisLength = 1.0;
    
    // X axis (red) - points RIGHT
    const xAxis = this.createArrow(0xff0000, axisLength);
    xAxis.rotation.z = -Math.PI / 2;
    this.deviceGroup.add(xAxis);
    
    // Y axis (green) - points UP (toward LED)
    const yAxis = this.createArrow(0x00ff00, axisLength);
    this.deviceGroup.add(yAxis);
    
    // Z axis (blue) - points OUT (front face)
    const zAxis = this.createArrow(0x0088ff, axisLength);
    zAxis.rotation.x = Math.PI / 2;
    this.deviceGroup.add(zAxis);
    
    // Axis labels
    this.addAxisLabel('X', new THREE.Vector3(axisLength + 0.15, 0, 0), 0xff0000);
    this.addAxisLabel('Y', new THREE.Vector3(0, axisLength + 0.15, 0), 0x00ff00);
    this.addAxisLabel('Z', new THREE.Vector3(0, 0, axisLength + 0.15), 0x0088ff);
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
   */
  private createReferenceFrame(): void {
    const refGroup = new THREE.Group();
    const axisLength = 2.0;
    const opacity = 0.3;
    
    // North (X) - red dashed
    const northGeometry = new THREE.BufferGeometry().setFromPoints([
      new THREE.Vector3(0, 0, 0),
      new THREE.Vector3(axisLength, 0, 0)
    ]);
    const northMaterial = new THREE.LineDashedMaterial({ 
      color: 0xff4444, 
      dashSize: 0.1, 
      gapSize: 0.05,
      opacity,
      transparent: true
    });
    const north = new THREE.Line(northGeometry, northMaterial);
    north.computeLineDistances();
    refGroup.add(north);
    
    // East (Y) - green dashed
    const eastGeometry = new THREE.BufferGeometry().setFromPoints([
      new THREE.Vector3(0, 0, 0),
      new THREE.Vector3(0, 0, axisLength)
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
    
    // Down (Z) - blue dashed
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
    
    // North label
    this.addWorldLabel('N', new THREE.Vector3(axisLength + 0.2, 0, 0));
    
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
   */
  setOrientation(q: Quaternion): void {
    // Three.js uses (x, y, z, w) order for quaternions
    this.deviceGroup.quaternion.set(q.x, q.y, q.z, q.w);
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
    
    // Slowly rotate camera around the scene when showing mag plot
    if (this.showingMagPlot) {
      const time = Date.now() * 0.0003;
      this.camera.position.x = Math.cos(time) * 4;
      this.camera.position.z = Math.sin(time) * 4;
      this.camera.lookAt(0, 0, 0);
    }
    
    this.renderer.render(this.scene, this.camera);
  }
  
  /**
   * Toggle magnetometer 3D scatter plot
   */
  toggleMagPlot(
    samples: MAGData[],
    applyTransform: boolean,
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
      // Transform to device frame if needed
      let x = applyTransform ? -sample.x : sample.x;
      let y = sample.y;
      let z = applyTransform ? -sample.z : sample.z;
      
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
   * Clean up resources
   */
  dispose(): void {
    if (this.animationId !== null) {
      cancelAnimationFrame(this.animationId);
    }
    this.renderer.dispose();
    this.container.removeChild(this.renderer.domElement);
  }
}
