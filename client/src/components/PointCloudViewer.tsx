import React from 'react';
import { useRecoilValue } from 'recoil';
import { Canvas } from '@react-three/fiber';
import { OrbitControls } from '@react-three/drei';

import PointCloudMesh from './PointCloudMesh';
import { pointsState, pointsType } from '../store/pointsState';

export type PointCloudViewerProps = {};

const PointCloudViewer: React.FC<PointCloudViewerProps> = () => {
  const points = useRecoilValue<pointsType>(pointsState);

  return (
    <Canvas
      camera={{ position: [0, 0, 15] }}
      style={{
        height: '800px',
        backgroundColor: 'white',
        borderColor: 'black',
        border: '2px solid',
      }}
    >
      <OrbitControls />
      <PointCloudMesh points={points} />
    </Canvas>
  );
};

export default PointCloudViewer;
