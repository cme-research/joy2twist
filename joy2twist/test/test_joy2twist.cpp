// Node-level tests for joy2twist: a test node publishes sensor_msgs/Joy and
// records the resulting Twist output and e-stop service calls.

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <functional>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "joy2twist/joy2twist_node.hpp"
#include "joy2twist/joy_input.hpp"

using namespace std::chrono_literals;
using joy2twist::JoyInput;
using joy2twist::Joy2TwistNode;
using joy2twist::MsgBool;
using joy2twist::MsgJoy;
using joy2twist::MsgTwist;
using joy2twist::SrvTrigger;

// --- JoyInput parsing ---------------------------------------------------------

TEST(JoyInput, ParsesAxesAndButtons)
{
  auto a = JoyInput::from_string("A3");
  EXPECT_EQ(a.type, JoyInput::Type::AXIS);
  EXPECT_EQ(a.index, 3);
  EXPECT_FALSE(a.is_inverted);

  auto b = JoyInput::from_string("B10");
  EXPECT_EQ(b.type, JoyInput::Type::BUTTON);
  EXPECT_EQ(b.index, 10);

  auto inv = JoyInput::from_string("!A5");
  EXPECT_EQ(inv.type, JoyInput::Type::AXIS);
  EXPECT_EQ(inv.index, 5);
  EXPECT_TRUE(inv.is_inverted);
}

TEST(JoyInput, RejectsMalformedStrings)
{
  for (const std::string s : {"", "A", "X1", "Bx", "!", "!C2"}) {
    EXPECT_THROW(JoyInput::from_string(s), std::invalid_argument) << '"' << s << '"';
  }
}

// --- Node behaviour -------------------------------------------------------------

class Joy2TwistTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}

  void start(std::vector<rclcpp::Parameter> params = {}, std::vector<std::string> args = {})
  {
    static int counter = 0;
    ns_ = "/j2t_test_" + std::to_string(counter++);
    params.emplace_back("cmd_vel_topic", ns_ + "/cmd_vel");

    std::vector<std::string> ros_args = {"--ros-args", "-r", "joy:=" + ns_ + "/joy"};
    ros_args.insert(ros_args.end(), args.begin(), args.end());
    rclcpp::NodeOptions options;
    options.parameter_overrides(params).arguments(ros_args);
    node_ = std::make_shared<Joy2TwistNode>(options);

    tester_ = std::make_shared<rclcpp::Node>("j2t_tester", ns_);
    joy_pub_ = tester_->create_publisher<MsgJoy>(ns_ + "/joy", rclcpp::SensorDataQoS());
    twist_sub_ = tester_->create_subscription<MsgTwist>(
      ns_ + "/cmd_vel", 10, [this](MsgTwist::SharedPtr m) {twists_.push_back(*m);});

    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_);
    executor_->add_node(tester_);
    // Let discovery settle.
    spin_for(300ms);
  }

  void TearDown() override
  {
    executor_.reset();
    node_.reset();
    tester_.reset();
  }

  void spin_for(std::chrono::milliseconds d)
  {
    const auto end = std::chrono::steady_clock::now() + d;
    while (std::chrono::steady_clock::now() < end) {
      executor_->spin_some(10ms);
    }
  }

  /// X-Input layout: 8 axes, 11 buttons.
  static MsgJoy joy(std::map<int, float> axes = {}, std::set<int> buttons = {})
  {
    MsgJoy msg;
    msg.axes.assign(8, 0.0f);
    msg.buttons.assign(11, 0);
    for (auto [i, v] : axes) {msg.axes.at(i) = v;}
    for (int b : buttons) {msg.buttons.at(b) = 1;}
    return msg;
  }

  /// Publish one joy message and return the twists it produced.
  std::vector<MsgTwist> send(const MsgJoy & msg, std::chrono::milliseconds wait = 300ms)
  {
    twists_.clear();
    joy_pub_->publish(msg);
    spin_for(wait);
    return twists_;
  }

  std::string ns_;
  std::shared_ptr<Joy2TwistNode> node_;
  rclcpp::Node::SharedPtr tester_;
  rclcpp::Executor::SharedPtr executor_;
  rclcpp::Publisher<MsgJoy>::SharedPtr joy_pub_;
  rclcpp::Subscription<MsgTwist>::SharedPtr twist_sub_;
  std::vector<MsgTwist> twists_;
};

// Default mapping: angular_z A2, linear_x A1, linear_y A0, dead man B4,
// fast B7, slow B5; factors fast 1.0 / regular 0.5 / slow 0.2.
constexpr int kDeadMan = 4;
constexpr int kSlow = 5;
constexpr int kFast = 7;

TEST_F(Joy2TwistTest, NoOutputWithoutDeadManSwitch)
{
  start();
  EXPECT_TRUE(send(joy({{1, 1.0f}, {2, 1.0f}})).empty());
}

TEST_F(Joy2TwistTest, RegularModeScalesAxes)
{
  start();
  auto out = send(joy({{0, -0.5f}, {1, 1.0f}, {2, 0.8f}}, {kDeadMan}));
  ASSERT_EQ(out.size(), 1u);
  EXPECT_FLOAT_EQ(out[0].linear.x, 0.5f);
  EXPECT_FLOAT_EQ(out[0].linear.y, -0.25f);
  EXPECT_FLOAT_EQ(out[0].angular.z, 0.4f);
}

TEST_F(Joy2TwistTest, SlowAndFastModes)
{
  start();
  auto slow = send(joy({{1, 1.0f}}, {kDeadMan, kSlow}));
  ASSERT_EQ(slow.size(), 1u);
  EXPECT_FLOAT_EQ(slow[0].linear.x, 0.2f);

  auto fast = send(joy({{1, 1.0f}}, {kDeadMan, kFast}));
  ASSERT_EQ(fast.size(), 1u);
  EXPECT_FLOAT_EQ(fast[0].linear.x, 1.0f);

  // Both pressed: ambiguous, falls back to regular.
  auto both = send(joy({{1, 1.0f}}, {kDeadMan, kSlow, kFast}));
  ASSERT_EQ(both.size(), 1u);
  EXPECT_FLOAT_EQ(both[0].linear.x, 0.5f);
}

TEST_F(Joy2TwistTest, ReleasingDeadManSendsExactlyOneStop)
{
  start();
  ASSERT_EQ(send(joy({{1, 1.0f}}, {kDeadMan})).size(), 1u);

  auto stop = send(joy({{1, 1.0f}}));
  ASSERT_EQ(stop.size(), 1u);
  EXPECT_EQ(stop[0], MsgTwist());

  EXPECT_TRUE(send(joy({{1, 1.0f}})).empty());
}

TEST_F(Joy2TwistTest, ShortJoyMessageDoesNotKillTheNode)
{
  start();
  ASSERT_EQ(send(joy({{1, 1.0f}}, {kDeadMan})).size(), 1u);

  MsgJoy short_msg;
  short_msg.axes = {0.0f, 1.0f};          // A2 (angular_z) is missing
  short_msg.buttons = {0, 0, 0, 0, 1};    // dead man pressed
  std::vector<MsgTwist> out;
  ASSERT_NO_THROW(out = send(short_msg));
  // The robot is stopped instead of the node dying.
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0], MsgTwist());

  // And it keeps working afterwards.
  auto again = send(joy({{1, 1.0f}}, {kDeadMan}));
  ASSERT_EQ(again.size(), 1u);
  EXPECT_FLOAT_EQ(again[0].linear.x, 0.5f);
}

TEST_F(Joy2TwistTest, ShippedUgvConfigMapsTriggersAndSticks)
{
  // config/joy2twist_ugv.yaml: angular_z A3, linear A1/A0, dead man B4 (LB),
  // fast '!A5' (RT pulled -> axis goes to -1), slow B5; regular 1.2 / fast 2.0.
  start({rclcpp::Parameter("e_stop.present", false)},
    {"--params-file", std::string(JOY2TWIST_CONFIG_DIR) + "/joy2twist_ugv.yaml"});

  auto regular = send(joy({{1, 0.5f}, {3, -1.0f}}, {4}));
  ASSERT_EQ(regular.size(), 1u);
  EXPECT_FLOAT_EQ(regular[0].linear.x, 0.6f);
  EXPECT_FLOAT_EQ(regular[0].angular.z, -0.78f);

  auto fast = send(joy({{1, 0.5f}, {5, -1.0f}}, {4}));
  ASSERT_EQ(fast.size(), 1u);
  EXPECT_FLOAT_EQ(fast[0].linear.x, 1.0f);
}

// --- e-stop -----------------------------------------------------------------------

class Joy2TwistEStopTest : public Joy2TwistTest
{
protected:
  // Default e-stop mapping: reset B1, trigger B2, enable reset B6.
  static constexpr int kReset = 1;
  static constexpr int kTrigger = 2;
  static constexpr int kEnableReset = 6;

  void start_with_e_stop()
  {
    start({
      rclcpp::Parameter("e_stop.present", true),
      rclcpp::Parameter("e_stop.topic", "/j2t_estop/e_stop"),
      rclcpp::Parameter("e_stop.reset_srv", "/j2t_estop/reset"),
      rclcpp::Parameter("e_stop.trigger_srv", "/j2t_estop/trigger"),
    });
    auto handler = [](std::atomic<int> & count) {
        return [&count](const std::shared_ptr<SrvTrigger::Request>,
                 std::shared_ptr<SrvTrigger::Response> res) {
                 ++count;
                 res->success = true;
               };
      };
    trigger_srv_ = tester_->create_service<SrvTrigger>(
      "/j2t_estop/trigger", handler(trigger_calls_));
    reset_srv_ = tester_->create_service<SrvTrigger>("/j2t_estop/reset", handler(reset_calls_));
    e_stop_pub_ = tester_->create_publisher<MsgBool>(
      "/j2t_estop/e_stop", rclcpp::QoS(1).transient_local().reliable());
    spin_for(300ms);
  }

  void set_e_stop(bool engaged)
  {
    MsgBool m;
    m.data = engaged;
    e_stop_pub_->publish(m);
    spin_for(300ms);
  }

  std::atomic<int> trigger_calls_{0};
  std::atomic<int> reset_calls_{0};
  rclcpp::Service<SrvTrigger>::SharedPtr trigger_srv_;
  rclcpp::Service<SrvTrigger>::SharedPtr reset_srv_;
  rclcpp::Publisher<MsgBool>::SharedPtr e_stop_pub_;
};

TEST_F(Joy2TwistEStopTest, TriggerStopsRobotAndCallsServiceBeforeAnyEStopState)
{
  start_with_e_stop();
  // No e-stop state received yet: the trigger must still stop and call the service.
  auto out = send(joy({}, {kTrigger}), 1000ms);
  ASSERT_FALSE(out.empty());
  EXPECT_EQ(out.front(), MsgTwist());
  EXPECT_EQ(trigger_calls_.load(), 1);
}

TEST_F(Joy2TwistEStopTest, TriggerIsIgnoredWhileEngaged)
{
  start_with_e_stop();
  set_e_stop(true);
  send(joy({}, {kTrigger}), 500ms);
  EXPECT_EQ(trigger_calls_.load(), 0);
}

TEST_F(Joy2TwistEStopTest, ResetNeedsEnableButtonAndEngagedEStop)
{
  start_with_e_stop();
  send(joy({}, {kReset, kEnableReset}), 500ms);
  EXPECT_EQ(reset_calls_.load(), 0) << "reset while not engaged";

  set_e_stop(true);
  send(joy({}, {kReset}), 500ms);
  EXPECT_EQ(reset_calls_.load(), 0) << "reset without enable button";

  send(joy({}, {kReset, kEnableReset}), 1000ms);
  EXPECT_EQ(reset_calls_.load(), 1);
}
