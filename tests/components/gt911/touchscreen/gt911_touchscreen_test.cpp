#include <gtest/gtest.h>

#include "esphome/core/application.h"
#include "esphome/components/gt911/touchscreen/gt911_touchscreen.h"
#include "../common.h"

namespace esphome::gt911::testing {

TEST(GT911TouchscreenTest, SetupCompletes) {
  RegisterBus bus;
  bus.registers[0x8048] = {0x40, 0x01, 0xF0, 0x00};
  GT911Touchscreen hub;
  hub.set_i2c_bus(&bus);
  hub.set_i2c_address(0x5D);
  hub.setup();
  EXPECT_TRUE(hub.can_proceed());
  EXPECT_FALSE(hub.is_failed());
}

TEST(GT911TouchscreenTest, SetupFailsWithoutTheChip) {
  RegisterBus bus;
  bus.fail = true;
  GT911Touchscreen hub;
  hub.set_i2c_bus(&bus);
  hub.set_i2c_address(0x5D);
  hub.setup();
  EXPECT_FALSE(hub.can_proceed());
  EXPECT_TRUE(hub.is_failed());
}

TEST(GT911TouchscreenTest, SleepIsIdempotentAndStopsTouchReads) {
  RegisterBus bus;
  GT911Touchscreen hub;
  hub.set_i2c_bus(&bus);
  hub.set_i2c_address(0x5D);
  hub.setup();
  bus.writes.clear();
  hub.sleep();
  hub.sleep();
  hub.update();
  hub.loop();
  ASSERT_EQ(bus.writes.size(), 1);
  EXPECT_EQ(bus.writes[0], (std::vector<uint8_t>{0x80, 0x40, 0x05}));
  EXPECT_FALSE(hub.is_failed());
}

class GT911SleepTest : public ::testing::Test {
 protected:
  void TearDown() override {
    App.~Application();
    new (&App) Application();
  }
  void run_timers_() {
    const uint32_t start = millis();
    while (millis() - start < 150) {
      delay(5);
      App.scheduler.call(millis());
    }
  }
};

TEST_F(GT911SleepTest, EarlySleepWaitsForSetupAndPollerStartup) {
  RegisterBus bus;
  GT911Touchscreen hub;
  hub.set_i2c_bus(&bus);
  hub.set_i2c_address(0x5D);
  hub.sleep();
  EXPECT_TRUE(bus.writes.empty());
  hub.call();
  ASSERT_TRUE(hub.can_proceed());
  bus.writes.clear();
  this->run_timers_();
  ASSERT_EQ(bus.writes.size(), 1);
  EXPECT_EQ(bus.writes[0], (std::vector<uint8_t>{0x80, 0x40, 0x05}));
  EXPECT_FALSE(App.scheduler.cancel_interval(&hub, InternalSchedulerID::POLLING_UPDATE));
  EXPECT_EQ(hub.get_component_state() & COMPONENT_STATE_MASK, COMPONENT_STATE_LOOP_DONE);
}

class RecordingPin final : public GPIOPin {
 public:
  void setup() override { this->setup_calls++; }
  void pin_mode(gpio::Flags flags) override { this->flags = flags; }
  gpio::Flags get_flags() const override { return this->flags; }
  bool digital_read() override { return this->level; }
  void digital_write(bool value) override {
    this->level = value;
    this->levels.push_back(value);
  }
  gpio::Flags flags{gpio::FLAG_INPUT};
  bool level{false};
  size_t setup_calls{0};
  std::vector<bool> levels;
};

TEST_F(GT911SleepTest, NoResetPinWakesChipBeforeReadingConfiguration) {
  RegisterBus bus;
  RecordingPin interrupt;
  GT911Touchscreen hub;
  hub.set_i2c_bus(&bus);
  hub.set_i2c_address(0x5D);
  hub.set_interrupt_pin(&interrupt);
  hub.setup();
  EXPECT_FALSE(hub.can_proceed());
  EXPECT_TRUE(bus.writes.empty());
  EXPECT_EQ(interrupt.levels, (std::vector<bool>{true}));
  EXPECT_EQ(interrupt.flags, gpio::FLAG_INPUT);
  this->run_timers_();
  EXPECT_TRUE(hub.can_proceed());
  EXPECT_FALSE(hub.is_failed());
}

TEST_F(GT911SleepTest, FailedSleepCanRetryWithoutResumingTouchReads) {
  RegisterBus bus;
  RecordingPin interrupt;
  GT911Touchscreen hub;
  hub.set_i2c_bus(&bus);
  hub.set_i2c_address(0x5D);
  hub.set_interrupt_pin(&interrupt);
  hub.call();
  this->run_timers_();
  ASSERT_TRUE(hub.can_proceed());
  bus.writes.clear();
  interrupt.levels.clear();
  const auto setup_calls = interrupt.setup_calls;
  bus.fail = true;
  hub.sleep();
  EXPECT_FALSE(hub.is_failed());
  EXPECT_TRUE(hub.status_has_warning());
  const auto transactions = bus.transactions;
  hub.update();
  hub.loop();
  EXPECT_EQ(bus.transactions, transactions);
  bus.fail = false;
  hub.sleep();
  EXPECT_FALSE(hub.status_has_warning());
  hub.sleep();
  EXPECT_EQ(bus.writes.size(), 2);
  EXPECT_EQ(interrupt.setup_calls, setup_calls);
  EXPECT_EQ(interrupt.flags, gpio::FLAG_OUTPUT);
  EXPECT_FALSE(interrupt.level);
  EXPECT_EQ(interrupt.levels, (std::vector<bool>{false, false, false, false}));
}

TEST_F(GT911SleepTest, SleepDuringResetWaitsAndPreservesResetSequence) {
  RegisterBus bus;
  RecordingPin interrupt;
  RecordingPin reset;
  GT911Touchscreen hub;
  hub.set_i2c_bus(&bus);
  hub.set_i2c_address(0x5D);
  hub.set_interrupt_pin(&interrupt);
  hub.set_reset_pin(&reset);
  hub.call();
  hub.sleep();
  EXPECT_FALSE(hub.can_proceed());
  EXPECT_TRUE(bus.writes.empty());
  EXPECT_EQ(reset.levels, (std::vector<bool>{false}));
  this->run_timers_();
  EXPECT_TRUE(hub.can_proceed());
  EXPECT_EQ(reset.levels, (std::vector<bool>{false, true}));
  EXPECT_EQ(interrupt.flags, gpio::FLAG_OUTPUT);
  EXPECT_FALSE(interrupt.level);
  ASSERT_FALSE(bus.writes.empty());
  EXPECT_EQ(bus.writes.back(), (std::vector<uint8_t>{0x80, 0x40, 0x05}));
  const auto transactions = bus.transactions;
  hub.sleep();
  hub.update();
  hub.loop();
  EXPECT_EQ(bus.transactions, transactions);
}

TEST_F(GT911SleepTest, FailedSetupDoesNotSendSleepCommand) {
  RegisterBus bus;
  bus.fail = true;
  GT911Touchscreen hub;
  hub.set_i2c_bus(&bus);
  hub.set_i2c_address(0x5D);
  hub.sleep();
  hub.call();
  EXPECT_TRUE(hub.is_failed());
  bus.writes.clear();
  this->run_timers_();
  hub.sleep();
  EXPECT_TRUE(bus.writes.empty());
}

}  // namespace esphome::gt911::testing
