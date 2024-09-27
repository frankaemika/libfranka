// Copyright (c) 2024 Franka Robotics GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <iostream>

#include <franka/logging/logger.hpp>
#include <franka/logging/logging_sink_interface.hpp>

using namespace std::string_literals;

class TestLogger : public franka::LoggingSinkInterface {
 public:
  TestLogger() = default;
  TestLogger(const std::string& name) : name(name) {}

  auto getName() const -> std::string override { return name; }

  auto logInfo(const std::string& message) -> void override {
    log_info_messages.push_back(message);
  }

  auto logWarn(const std::string& message) -> void override {
    log_warn_messages.push_back(message);
  }

  auto logError(const std::string& message) -> void override {
    log_error_messages.push_back(message);
  }

  std::vector<std::string> log_info_messages;
  std::vector<std::string> log_warn_messages;
  std::vector<std::string> log_error_messages;

 private:
  std::string name = "TestLogger";
};

class LoggingTests : public ::testing::Test {
 public:
  void SetUp() override { franka::logging::addLogger(test_logger); }

  void TearDown() override { franka::logging::removeAllLoggers(); }

 protected:
  std::shared_ptr<TestLogger> test_logger = std::make_shared<TestLogger>();
};

TEST_F(LoggingTests, givenLogger_whenLogInfo_thenMessageLogged) {
  auto log_message = "TestInfo"s;
  franka::logging::logInfo(log_message);

  ASSERT_EQ(test_logger->log_info_messages.size(), 1u);
  ASSERT_EQ(test_logger->log_info_messages[0], log_message);
}

TEST_F(LoggingTests, givenLogger_whenLogWarn_thenMessageLogged) {
  auto log_message = "TestWarning"s;
  franka::logging::logWarn(log_message);

  ASSERT_EQ(test_logger->log_warn_messages.size(), 1u);
  ASSERT_EQ(test_logger->log_warn_messages[0], log_message);
}

TEST_F(LoggingTests, givenLogger_whenLogError_thenMessageLogged) {
  auto log_message = "TestError"s;
  franka::logging::logError(log_message);

  ASSERT_EQ(test_logger->log_error_messages.size(), 1u);
  ASSERT_EQ(test_logger->log_error_messages[0], log_message);
}

TEST_F(LoggingTests, givenLogger_whenRemove_thenNoMessagesLogged) {
  auto log_message = "TestInfo"s;
  franka::logging::removeLogger(test_logger->getName());

  franka::logging::logInfo(log_message);
  franka::logging::logWarn(log_message);
  franka::logging::logError(log_message);

  ASSERT_EQ(test_logger->log_info_messages.size(), 0u);
  ASSERT_EQ(test_logger->log_warn_messages.size(), 0u);
  ASSERT_EQ(test_logger->log_error_messages.size(), 0u);
}

TEST_F(LoggingTests, givenMultipleLoggers_whenLog_thenAllMessagesLogged) {
  auto additional_logger = std::make_shared<TestLogger>("AdditionalLogger");
  franka::logging::addLogger(additional_logger);

  auto log_message = "TestInfo"s;
  franka::logging::logInfo(log_message);

  ASSERT_EQ(test_logger->log_info_messages.size(), 1u);
  ASSERT_EQ(additional_logger->log_info_messages.size(), 1u);
}

TEST_F(LoggingTests, givenMultipleLoggers_whenRemoveOne_thenOnlyOneLoggerLogs) {
  auto additional_logger = std::make_shared<TestLogger>("AdditionalLogger");
  franka::logging::addLogger(additional_logger);

  franka::logging::removeLogger(additional_logger->getName());

  auto log_message = "TestInfo"s;
  franka::logging::logInfo(log_message);

  ASSERT_EQ(test_logger->log_info_messages.size(), 1u);
  ASSERT_EQ(additional_logger->log_info_messages.size(), 0u);
}

TEST_F(LoggingTests, givenFMTLogLine_thenExpectedMessageLogged) {
  auto log_message1 = "Test Message";
  auto log_message2 = 3.0;
  auto resulting_string = fmt::format("{} {}", log_message1, log_message2);

  franka::logging::logInfo("{} {}", log_message1, log_message2);

  ASSERT_EQ(test_logger->log_info_messages.size(), 1u);
  ASSERT_EQ(test_logger->log_info_messages[0], resulting_string);
}
