import React, { useEffect, useState } from 'react';
import { w3cwebsocket } from 'websocket';
import { useRecoilValue, useSetRecoilState, useRecoilState } from 'recoil';
import { Button, Input } from '@material-ui/core';
import Box from '@material-ui/core/Box';
import Typography from '@material-ui/core/Typography';
import SyncIcon from '@material-ui/icons/Sync';
import SyncDisabledIcon from '@material-ui/icons/SyncDisabled';

import NavBar from './components/NavBar';
import PointCloudViewer from './components/PointCloudViewer';
import PointCloudReceiver from './components/PointCloudReceiver';
import { urlState, urlType } from './store/urlState';
import { pointsState, pointsType } from './store/pointsState';
import { connectState, connectType } from './store/connectState';

const App: React.FC = () => {
  // const url = useRecoilValue<urlType>(urlState);
  const connect = useRecoilValue<connectType>(connectState);
  const setPoints = useSetRecoilState(pointsState);
  const [client, setClient] = useState<w3cwebsocket>(
    undefined as unknown as w3cwebsocket
  );
  const [url, setUrl] = useRecoilState<urlType>(urlState);

  const updateAddress = (e: any) => {
    if (e.target.value.length > 39) return;
    setUrl({ address: e.target.value, port: url.port });
  };

  const updatePort = (e: any) => {
    if (e.target.value < 0 || 65538 < e.target.value) return;
    setUrl({ address: url.address, port: e.target.value });
  };

  useEffect(() => {
    setClient(new w3cwebsocket(`ws://${url.address}:${url.port}`));
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, []);

  const handleReConnect = () =>
    setClient(new w3cwebsocket(`ws://${url.address}:${url.port}`));

  const handleGetPoints = () => client?.send('');
  const clearPoints = () => setPoints([] as pointsType);

  return (
    <>
      <NavBar />
      <Box m={3}>
        <Button variant="contained" onClick={handleGetPoints}>
          GET POINTS
        </Button>
        <Button variant="contained" onClick={clearPoints}>
          CLEAR POINTS
        </Button>
        <Button variant="contained" onClick={handleReConnect}>
          RECONNECT
        </Button>
      </Box>
      <Box m={3}>
        <Typography variant="h6" noWrap>
          ws://
          <Input
            value={url.address}
            onChange={updateAddress}
            placeholder="address"
            inputProps={{ 'aria-label': 'address' }}
            style={{
              width: `${Math.max(9, url.address.length) * 9}px`,
              paddingLeft: '5px',
            }}
          />
          :
          <Input
            value={url.port}
            onChange={updatePort}
            placeholder="port"
            inputProps={{ 'aria-label': 'port' }}
            style={{
              width: `${Math.max(5, url.port.length) * 9}px`,
              paddingLeft: '5px',
            }}
          />
          {connect ? <SyncIcon /> : <SyncDisabledIcon />}
        </Typography>
      </Box>

      <PointCloudViewer />
      {client ? <PointCloudReceiver websock={client} /> : null}
    </>
  );
};

export default App;
