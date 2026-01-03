/**
 * Curved Arrow for visualizing angular velocity (gyro) data
 * 
 * Creates an arc around an axis with an arrowhead indicating rotation direction.
 * Uses TubeGeometry for consistent thickness across all WebGL contexts.
 */

import * as THREE from 'three';

export interface CurvedArrowOptions {
  /** Radius of the arc from the axis */
  radius?: number;
  /** Number of segments in the arc (smoothness) */
  segments?: number;
  /** Size of the arrowhead cone */
  headLength?: number;
  /** Radius of the arrowhead cone base */
  headRadius?: number;
  /** Thickness of the arc tube (radius of the tube cross-section) */
  tubeRadius?: number;
}

const DEFAULT_OPTIONS: Required<CurvedArrowOptions> = {
  radius: 0.8,
  segments: 32,
  headLength: 0.15,
  headRadius: 0.06,
  tubeRadius: 0.02  // Tube thickness - similar to ArrowHelper shaft
};

/**
 * Custom curve class for the arc path
 */
class ArcCurve extends THREE.Curve<THREE.Vector3> {
  private axis: 'x' | 'y' | 'z';
  private arcRadius: number;
  private startAngle: number;
  private endAngle: number;
  
  constructor(axis: 'x' | 'y' | 'z', radius: number, startAngle: number, endAngle: number) {
    super();
    this.axis = axis;
    this.arcRadius = radius;
    this.startAngle = startAngle;
    this.endAngle = endAngle;
  }
  
  getPoint(t: number): THREE.Vector3 {
    const theta = this.startAngle + t * (this.endAngle - this.startAngle);
    
    switch (this.axis) {
      case 'x':
        return new THREE.Vector3(0, this.arcRadius * Math.cos(theta), this.arcRadius * Math.sin(theta));
      case 'y':
        return new THREE.Vector3(this.arcRadius * Math.sin(theta), 0, this.arcRadius * Math.cos(theta));
      case 'z':
        return new THREE.Vector3(this.arcRadius * Math.cos(theta), this.arcRadius * Math.sin(theta), 0);
    }
  }
}

/**
 * Creates a curved arrow (arc with arrowhead) around a specified axis
 * 
 * @param axis - 'x', 'y', or 'z' - the axis to rotate around
 * @param angle - The arc angle in radians (positive = CCW when looking down axis)
 * @param color - The color of the arrow
 * @param options - Optional styling parameters
 * @returns A THREE.Group containing the arc and arrowhead
 */
export function createCurvedArrow(
  axis: 'x' | 'y' | 'z',
  angle: number,
  color: number,
  options: CurvedArrowOptions = {}
): THREE.Group {
  const opts = { ...DEFAULT_OPTIONS, ...options };
  const group = new THREE.Group();
  
  // Clamp angle to reasonable range for visibility
  const minAngle = 0.1;  // Minimum visible arc
  const maxAngle = Math.PI * 1.5;  // 270 degrees max
  const clampedAngle = Math.sign(angle) * Math.min(Math.max(Math.abs(angle), minAngle), maxAngle);
  
  // If angle is essentially zero, return empty group
  if (Math.abs(angle) < 0.01) {
    return group;
  }
  
  // Create the arc using TubeGeometry for consistent thickness
  const arcCurve = new ArcCurve(axis, opts.radius, 0, clampedAngle);
  const tubeGeometry = new THREE.TubeGeometry(
    arcCurve,
    opts.segments,      // tubular segments
    opts.tubeRadius,    // radius of tube
    8,                  // radial segments
    false               // not closed
  );
  const tubeMaterial = new THREE.MeshBasicMaterial({ color });
  const tube = new THREE.Mesh(tubeGeometry, tubeMaterial);
  group.add(tube);
  
  // Create arrowhead cone at the end of the arc
  const coneGeometry = new THREE.ConeGeometry(opts.headRadius, opts.headLength, 8);
  const coneMaterial = new THREE.MeshBasicMaterial({ color });
  const cone = new THREE.Mesh(coneGeometry, coneMaterial);
  
  // Position cone at the end of the arc
  const endPoint = arcCurve.getPoint(1);
  cone.position.copy(endPoint);
  
  // Orient cone tangent to the arc (pointing in direction of rotation)
  // Tangent direction depends on axis and sign of angle
  const tangent = getTangentDirection(axis, clampedAngle);
  
  // Cone points along +Y by default, so we need to rotate it to point along tangent
  const up = new THREE.Vector3(0, 1, 0);
  const quaternion = new THREE.Quaternion().setFromUnitVectors(up, tangent);
  cone.setRotationFromQuaternion(quaternion);
  
  group.add(cone);
  
  return group;
}

/**
 * Get the tangent direction at the end of the arc
 */
function getTangentDirection(axis: 'x' | 'y' | 'z', angle: number): THREE.Vector3 {
  const sign = Math.sign(angle);
  const theta = angle;
  
  // Tangent is derivative of position with respect to angle
  let tangent: THREE.Vector3;
  switch (axis) {
    case 'x':
      // d/dθ (0, r*cos(θ), r*sin(θ)) = (0, -r*sin(θ), r*cos(θ))
      tangent = new THREE.Vector3(0, -Math.sin(theta), Math.cos(theta));
      break;
    case 'y':
      // d/dθ (r*sin(θ), 0, r*cos(θ)) = (r*cos(θ), 0, -r*sin(θ))
      tangent = new THREE.Vector3(Math.cos(theta), 0, -Math.sin(theta));
      break;
    case 'z':
      // d/dθ (r*cos(θ), r*sin(θ), 0) = (-r*sin(θ), r*cos(θ), 0)
      tangent = new THREE.Vector3(-Math.sin(theta), Math.cos(theta), 0);
      break;
  }
  
  return tangent.normalize().multiplyScalar(sign);
}

/**
 * Disposes of all geometries and materials in a curved arrow group
 */
export function disposeCurvedArrow(group: THREE.Group): void {
  group.traverse((child) => {
    if (child instanceof THREE.Mesh) {
      child.geometry.dispose();
      (child.material as THREE.Material).dispose();
    }
  });
}
