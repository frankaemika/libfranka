#pragma once

#include <research_interface/types.h>

class CommandHandler {
 public:
  virtual void handleStartMotionGeneratorReply(
      const research_interface::StartMotionGeneratorReply& reply) = 0;
  virtual void handleStopMotionGeneratorReply(
      const research_interface::StopMotionGeneratorReply& reply) = 0;
  virtual void handleStartControllerReply(
      const research_interface::StartControllerReply& reply) = 0;
  virtual void handleStopControllerReply(
      const research_interface::StopControllerReply& reply) = 0;
};