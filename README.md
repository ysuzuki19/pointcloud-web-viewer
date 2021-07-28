# Point Cloud Web Viewer

pointcloud viewer on browser via websocket.

# Run Server

Prepare a PCD file for anything,
Put `server/echo/data.pcd`.

```bash
$ cd server/echo
$ mkdir build
$ cd build
$ cmake ..
$ make
$ ./server 0.0.0.0 8080
```

# Run Client

Before run client, please run server.

```bash
$ cd client
$ npm install
$ npm start
```

# Demo

## Echo

![files](./demo/point-cloud-web-viewer-simple-demo.gif)

## Stream

![files](./demo/point-cloud-web-viewer-stream-demo.gif)

# Run Stream Server

Prepare PCD files and put `server/streamer/pcds/<filenam>.pcd`.

```bash
$ cd server/streamer
$ mkdir build
$ cd build
$ cmake ..
$ make
$ ./server 0.0.0.0 8080
```
