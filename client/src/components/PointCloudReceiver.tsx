import React, { useEffect } from 'react';
import { useSetRecoilState } from 'recoil';
import { w3cwebsocket } from 'websocket';

import { pointsState } from '../store/pointsState';

export type messageType = {
  points?: Array<[number, number, number]>;
};

export type messageDataType = messageType & {
  type: string;
};

export type PointCloudReceiverProps = {
  websock: w3cwebsocket;
};

const PointCloudReceiver: React.FC<PointCloudReceiverProps> = ({ websock }) => {
  const setPoints = useSetRecoilState<any>(pointsState);

  useEffect(() => {
    websock.onopen = () => {
      console.log('WebSocket  Connected');
    };
    websock.onclose = () => {
      console.log('WebSocket Client Disconnected');
    };
    websock.onmessage = (event: any) => {
      const dataFromServer: messageDataType = JSON.parse(event.data);
      if (dataFromServer.type === 'points') {
        setPoints(dataFromServer.points);
      }
    };

    return () => {
      websock.close();
    };
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [websock]);

  return <></>;
};

export default PointCloudReceiver;
