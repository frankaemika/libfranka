#include "mock_server.h"
#include <iostream>
#include <chrono>
#include <thread>
#include <mutex>
#include <cassert>

using namespace std::chrono_literals;

using move_t = research_interface::robot::Move;
using ctrl_mode_t = move_t::ControllerMode;
using motion_mode_t = move_t::MotionGeneratorMode;

struct Modes {
  std::mutex mut;
  research_interface::robot::ControllerMode ctrl_mode;
  research_interface::robot::MotionGeneratorMode motion_mode;
  research_interface::robot::RobotMode robot_mode;
};

research_interface::robot::ControllerMode convert_mode(const ctrl_mode_t & mode) {

  using md_t = research_interface::robot::ControllerMode;

  switch (mode) {
    case ctrl_mode_t::kCartesianImpedance:
      return md_t::kCartesianImpedance;
    case ctrl_mode_t::kExternalController:
      return md_t::kExternalController;
    case ctrl_mode_t::kJointImpedance:
      return md_t::kJointImpedance;
    default:
      assert(false);
      return md_t::kOther;
  }
}

research_interface::robot::MotionGeneratorMode convert_mode(const motion_mode_t & mode) {

  using md_t = research_interface::robot::MotionGeneratorMode;

  switch (mode) {
    case motion_mode_t::kCartesianPosition :
      return md_t::kCartesianPosition;
    case motion_mode_t::kCartesianVelocity:
      return md_t::kCartesianVelocity;
    case motion_mode_t::kJointPosition:
      return md_t::kJointPosition;
    case motion_mode_t::kJointVelocity:
      return md_t::kJointVelocity;
    default:
      return md_t::kIdle;
  }
}

research_interface::robot::Move::Response moveResp() {
  return research_interface::robot::Move::Response(research_interface::robot::Move::Status::kSuccess);
}

RobotTypes::Connect::Response connSuccessResponse() {
  return RobotTypes::Connect::Response(RobotTypes::Connect::Status::kSuccess);
}

void send_state(MockServer<RobotTypes>& server, Modes& modes) {
  using clock = std::chrono::steady_clock;
  research_interface::robot::RobotState state{};

  std::unique_lock<std::mutex> modes_lck(modes.mut);
  state.controller_mode = modes.ctrl_mode;
  state.motion_generator_mode = modes.motion_mode;
  state.robot_mode = modes.robot_mode;
  modes_lck.unlock();

  auto time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(clock::now().time_since_epoch()).count();

  state.message_id = time_ms;

  server.sendStateSync<research_interface::robot::RobotState>(state);

}

void send_states_thread(MockServer<RobotTypes>& server, uint64_t dt_us, Modes& modes) {
  using clock = std::chrono::steady_clock;
  const auto sleep_dt = 50us;

  auto t_next = clock::now() + std::chrono::microseconds(dt_us);

  while (true) {

    send_state(server, modes);

    while (clock::now() < t_next) {
      std::this_thread::sleep_for(sleep_dt);
    }
    t_next += std::chrono::microseconds(dt_us);
  }
}

int main(int argc, char* argv[]) {

  bool connected = false;
  bool cmd_received = false;
  std::condition_variable cv;
  std::mutex mut;

  Modes modes;

  auto onConn = [&](const RobotTypes::Connect::Request & req) {
    std::cout << "Connected! port: " << req.udp_port << std::endl;


    std::unique_lock<std::mutex> lck(mut);
    connected = true;
    cv.notify_one();


    auto status = RobotTypes::Connect::Status::kSuccess;
    return RobotTypes::Connect::Response(status);
  };

  MockServer<RobotTypes> server(onConn);

  std::unique_lock<std::mutex> lck(mut);
  while (!connected) {
    cv.wait(lck);
  }

  lck.unlock();

  auto state_thread = std::thread([&]{send_states_thread(server,1000, modes);});

  uint32_t cmd_id;
  //server.queueResponse<research_interface::robot::Move>(0, []{return moveResp();}).spinOnce();

  //server.waitForCommand<research_interface::robot::Move>([&](const auto& x){
  //  std::cout << "Received Move" << std::endl;
  //  return research_interface::robot::Move::Response(research_interface::robot::Move::Status::kSuccess);
  //}, &cmd_id).spinOnce();
  server.waitForCommand<research_interface::robot::Move>([&](const research_interface::robot::Move::Request& cmd){
    std::cout << "Received TCP Move" << std::endl;

    std::unique_lock<std::mutex> modes_lck(modes.mut);
    modes.ctrl_mode = convert_mode(cmd.controller_mode);
    modes.motion_mode = convert_mode(cmd.motion_generator_mode);
    modes.robot_mode = research_interface::robot::RobotMode::kMove;
    modes_lck.unlock();
    
    std::unique_lock<std::mutex> lck(mut);
    cmd_received = true;
    cv.notify_one();
    lck.unlock();

    return moveResp();
  }, &cmd_id).spinOnce();

  lck.lock();
  while (!cmd_received) {
    cv.wait(lck);
  }
  cmd_received = false;
  lck.unlock();

  send_state(server, modes);

  server.sendResponse<research_interface::robot::Move>(cmd_id, moveResp).spinOnce();
  //server.sendResponse<research_interface::robot::Move>(cmd_id, moveResp).spinOnce();
  //server.sendResponse<research_interface::robot::Move>(cmd_id, moveResp).spinOnce();

  while (true) {
    server.onReceiveRobotCommand([&](const research_interface::robot::RobotCommand& cmd){
      //std::cout << "Received UDP Move" << std::endl;
      cmd_id = cmd.message_id;
      std::unique_lock<std::mutex> lck(mut);
      cmd_received = true;
      cv.notify_one();
      lck.unlock();


      server.sendResponse<research_interface::robot::Move>(cmd_id, moveResp);
    }).spinOnce();

    //std::this_thread::sleep_for(std::chrono::microseconds(500));

    lck.lock();
    while (!cmd_received) {
      cv.wait(lck);
    }
    cmd_received = false;
    lck.unlock();

  }
}