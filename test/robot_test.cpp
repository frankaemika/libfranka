#include <gtest/gtest.h>

#include <boost/asio.hpp>
#include <thread>
#include <chrono>

#include "franka/robot.h"
#include "../src/robot_service/messages.h"

//TEST(Robot, CanConstruct) {
//  franka::Robot robot("unit-test");
//}
//
//TEST(Robot, WaitForRobotReturnsFalse) {
//  franka::Robot robot("unit-test");
//  ASSERT_FALSE(robot.waitForRobotState());
//}
//
//TEST(Robot, CanGetRobotState) {
//  franka::Robot robot("unit-test");
//  franka::RobotState state = robot.getRobotState();
//}

TEST(Robot, CanPerformTCPHandshake) {
  using namespace std::chrono_literals;

  std::thread server_thread([]{
    using boost_tcp = boost::asio::ip::tcp;
    boost::asio::io_service io_service;
    boost_tcp::acceptor acceptor(io_service, boost_tcp::endpoint(boost_tcp::v4(), 1337));
    boost_tcp::socket socket(io_service);
    acceptor.accept(socket);
    char data[sizeof(robot_service::RIConnectRequest)];
    boost::asio::read(socket, boost::asio::buffer(data, sizeof(data)));
    robot_service::RIConnectReply reply;
    reply.ri_version = 1;
    reply.status_code = robot_service::StatusCode::kSuccess;
    unsigned char* reply_data = (unsigned char*)&reply;
    boost::asio::write(socket, boost::asio::buffer(reply_data, sizeof(reply)));
  });

  std::this_thread::sleep_for(0.2s);

  franka::Robot robot("localhost");

  server_thread.join();
}

TEST(Robot, ThrowOnIncompatibleLibraryVersion) {
  using namespace std::chrono_literals;

  std::thread server_thread([]{
    using boost_tcp = boost::asio::ip::tcp;
    boost::asio::io_service io_service;
    boost_tcp::acceptor acceptor(io_service, boost_tcp::endpoint(boost_tcp::v4(), 1337));
    boost_tcp::socket socket(io_service);
    acceptor.accept(socket);
    char data[sizeof(robot_service::RIConnectRequest)];
    boost::asio::read(socket, boost::asio::buffer(data, sizeof(data)));
    robot_service::RIConnectReply reply;
    reply.ri_version = 1;
    reply.status_code = robot_service::StatusCode::kIncompatibleLibraryVersion;
    unsigned char* reply_data = (unsigned char*)&reply;
    boost::asio::write(socket, boost::asio::buffer(reply_data, sizeof(reply)));
  });

  std::this_thread::sleep_for(0.2s);

  try {
    franka::Robot robot("localhost");
    FAIL() << "Expected franka::ProtocolException";
  }
  catch(franka::ProtocolException const & err) {
    EXPECT_EQ(err.what(),std::string("libfranka: incompatible library version"));
  }
  catch(...) {
    FAIL() << "Expected franka::ProtocolException";
  }

  server_thread.join();
}