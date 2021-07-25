import React, { useEffect, useState } from 'react';
import { w3cwebsocket } from 'websocket';
import { useRecoilValue, useSetRecoilState } from 'recoil';
import { Box, Button } from '@material-ui/core';

import NavBar from './components/NavBar';
import PointCloudViewer from './components/PointCloudViewer';
import PointCloudReceiver from './components/PointCloudReceiver';
import { urlState } from './store/urlState';
import { pointsState, pointsType } from './store/pointsState';

const App: React.FC = () => {
  const url = useRecoilValue(urlState);
  const setPoints = useSetRecoilState(pointsState);
  const [client, setClient] = useState<w3cwebsocket>(
    undefined as unknown as w3cwebsocket
  );

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
      <PointCloudViewer />
      {client ? <PointCloudReceiver websock={client} /> : null}
    </>
  );
};

export default App;
