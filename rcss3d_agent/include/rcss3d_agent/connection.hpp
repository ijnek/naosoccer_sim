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