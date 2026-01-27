/**
 * Shaded Arrow for visualizing vectors with proper lighting response
 * 
 * Replaces THREE.ArrowHelper with a custom implementation using MeshPhongMaterial
 * for consistent shading with the rest of the scene.
 */

import * as THREE from 'three';

export interface ShadedArrowOptions {
  /** Shaft radius */
  shaftRadius?: number;
  /** Head length */
  headLength?: number;
  /** Head radius */
  headRadius?: number;
  /** Number of segments for cylinder/cone (smoothness) */
  segments?: number;
  /** Specular highlight color */
  specular?: number;
  /** Shininess (0-100) */
  shininess?: number;
}

const DEFAULT_OPTIONS: Required<ShadedArrowOptions> = {
  shaftRadius: 0.02,
  headLength: 0.2,
  headRadius: 0.06,
  segments: 12,
  specular: 0x444444,
  shininess: 30
};

/**
 * Creates a shaded arrow with MeshPhongMaterial for proper lighting response
 * 
 * @param direction - Direction vector (will be normalized)
 * @param origin - Origin point of the arrow
 * @param length - Total length of the arrow
 * @param color - Arrow color
 * @param options - Optional styling parameters
 * @returns A THREE.Group containing the shaft and arrowhead
 */
export function createShadedArrow(
  direction: THREE.Vector3,
  origin: THREE.Vector3,
  length: number,
  color: number,
  options: ShadedArrowOptions = {}
): THREE.Group {
  const opts = { ...DEFAULT_OPTIONS, ...options };
  const group = new THREE.Group();
  
  // Normalize direction
  const dir = direction.clone().normalize();
  
  // Calculate shaft length (total length minus head)
  const shaftLength = Math.max(0.01, length - opts.headLength);
  
  // Create shared material for both shaft and head
  const material = new THREE.MeshPhongMaterial({
    color,
    specular: opts.specular,
    shininess: opts.shininess
  });
  
  // Create shaft (cylinder)
  const shaftGeometry = new THREE.CylinderGeometry(
    opts.shaftRadius,
    opts.shaftRadius,
    shaftLength,
    opts.segments
  );
  const shaft = new THREE.Mesh(shaftGeometry, material);
  // Position shaft so it starts at origin
  shaft.position.y = shaftLength / 2;
  group.add(shaft);
  
  // Create arrowhead (cone)
  const headGeometry = new THREE.ConeGeometry(
    opts.headRadius,
    opts.headLength,
    opts.segments
  );
  const head = new THREE.Mesh(headGeometry, material);
  // Position head at end of shaft
  head.position.y = shaftLength + opts.headLength / 2;
  group.add(head);
  
  // Orient the group to point along direction
  // Default orientation is +Y, we need to rotate to point along 'dir'
  const up = new THREE.Vector3(0, 1, 0);
  const quaternion = new THREE.Quaternion().setFromUnitVectors(up, dir);
  group.setRotationFromQuaternion(quaternion);
  
  // Position at origin
  group.position.copy(origin);
  
  return group;
}

/**
 * Disposes of all geometries and materials in a shaded arrow group
 */
export function disposeShadedArrow(group: THREE.Group): void {
  group.traverse((child) => {
    if (child instanceof THREE.Mesh) {
      child.geometry.dispose();
      if (child.material instanceof THREE.Material) {
        child.material.dispose();
      }
    }
  });
}
