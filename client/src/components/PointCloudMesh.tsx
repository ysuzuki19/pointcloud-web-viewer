import React, { useState, useEffect, useRef } from 'react';
import * as THREE from 'three';

import { pointType, pointsType } from '../store/pointsState';

export type PointCloudMeshProps = {
  points: pointsType;
};

const PointCloudMesh: React.FC<PointCloudMeshProps> = ({ points }) => {
  const ref = useRef<THREE.InstancedMesh>();
  const [beforeSize, setBeforeSize] = useState<number>(0);

  const clearMatrix = () => {
    for (let i = 0; i < beforeSize; ++i) {
      ref.current?.setMatrixAt(i, new THREE.Matrix4());
    }
  };

  useEffect(() => {
    if (ref && ref.current) {
      const transform = new THREE.Matrix4();
      clearMatrix();
      points.forEach((point: pointType, i: number) => {
        transform.setPosition(point[0], point[1], point[2]);
        ref.current?.setMatrixAt(i, transform);
      });
      ref.current.instanceMatrix.needsUpdate = true;
      setBeforeSize(points.length);
    }
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [points]);

  return (
    <instancedMesh ref={ref} args={[undefined, undefined, 10000]}>
      <sphereGeometry attach="geometry" args={[0.01, 16, 16]} />
      <meshStandardMaterial attach="material" color="black" />
    </instancedMesh>
  );
};

export default PointCloudMesh;
