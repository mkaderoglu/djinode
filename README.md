# djinode
Using NODE, you can integrate your web applications with the DJI OBOARD SDK. By editing the DJI OBOARD SDK, I integrated SOCKET.IO into the program and enabled the program to provide multiple connections, not just a single device connection.

I used https://github.com/socketio/socket.io-client-cpp and i used https://github.com/dji-sdk/Onboard-SDK

git clone https://github.com/socketio/socket.io-client-cpp
mkdir build
cd build
make

git clone https://github.com/dji-sdk/Onboard-SDK
mkdir build
cd build
make



# djinode C++ Socket.io and Dji Onboard SDK

Using NODE, you can integrate your web applications with the DJI OBOARD SDK. By editing the DJI OBOARD SDK, I integrated SOCKET.IO into the program and enabled the program to provide multiple connections, not just a single device connection.

## Features

- 100% written in modern C++11
- Compatible with socket.io 1.0+ protocol
- DJI Onboard SDK integration
- Multiple drone control
- Integration image processing

## Installation alternatives

* [With CMAKE](./INSTALL.md#with-cmake)
* Option 1: Create a static library
* Option 2: Manual integration


## Quickstart

** [Full overview of API can be seen here](./API.md) **

## License

MIT


