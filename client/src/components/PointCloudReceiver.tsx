import React, { useEffect } from 'react';
import { useSetRecoilState } from 'recoil';
import { w3cwebsocket } from 'websocket';

import { pointsState, pointsType } from '../store/pointsState';
import { connectState, connectType } from '../store/connectState';

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
  const setPoints = useSetRecoilState<pointsType>(pointsState);
  const setConnect = useSetRecoilState<connectType>(connectState);

  useEffect(() => {
    websock.onopen = () => {
      console.log('WebSocket  Connected');
      setConnect(true);
    };
    websock.onclose = () => {
      console.log('WebSocket Client Disconnected');
      setConnect(false);
    };
    websock.onmessage = (event: any) => {
      const dataFromServer: messageDataType = JSON.parse(event.data);
      if (dataFromServer.type === 'points') {
        setPoints(dataFromServer.points as pointsType);
      }
    };

    return () => {
      websock.close();
      setConnect(false);
    };
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [websock]);

  return <></>;
};

export default PointCloudReceiver;
