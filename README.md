# djinode
Using NODE, you can integrate your web applications with the DJI OBOARD SDK.

By editing the DJI OBOARD SDK, I integrated SOCKET.IO into the program and enabled the program to provide multiple connections, not just a single device connection.

I used https://github.com/socketio/socket.io-client-cpp and i used https://github.com/dji-sdk/Onboard-SDK

git clone https://github.com/socketio/socket.io-client-cpp
mkdir build
cd build
make

git clone https://github.com/dji-sdk/Onboard-SDK
mkdir build
cd build
make


