// Copyright 2021 Kenji Brameld
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef NAOSOCCER_SIM__CONNECTION_HPP
#define NAOSOCCER_SIM__CONNECTION_HPP

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "rcss3d_agent/socket.hpp"

class Connection
{
public:
    Connection();

    void initialise(const std::string &host, int port);

    void send(std::string msg);
    std::string receive();

private:
    void initSocket(std::string const &host, int port);
    void connect();
    void initConnection();
    uint32_t receive_();

    rclcpp::Logger logger;

    Socket socket_;
    SocketAddress socket_address_;

    std::vector<char> buffer_;
};

#endif // NAOSOCCER_SIM__CONNECTION_HPP