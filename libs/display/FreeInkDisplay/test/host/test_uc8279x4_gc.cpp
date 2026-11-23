#include <algorithm>
#include <cassert>
#include <vector>
// Inspect private state without adding test-only methods to the SDK API.
#define private public
#include "FreeInkDisplay.h"
#include "src/driver/Uc8279X4Driver.h"
#undef private
#include "BoardConfig.h"
#include "src/lut/Uc8279X4GcLuts.h"

using namespace freeink;
using Bytes = std::vector<uint8_t>;

static uint8_t lastRegister(const EpdBus& bus, uint8_t command) {
  for (auto i = bus.writes.rbegin(); i != bus.writes.rend(); ++i)
    if (i->command == command && !i->bytes.empty()) return i->bytes[0];
  assert(false);
  return 0;
}

static const Bytes& lastPlane(const EpdBus& bus, uint8_t command) {
  for (auto i = bus.writes.rbegin(); i != bus.writes.rend(); ++i)
    if (i->command == command) return i->bytes;
  assert(false);
  static const Bytes empty;
  return empty;
}

static Bytes frame(unsigned seed) {
  Bytes b(48000);
  for (size_t i = 0; i < b.size(); ++i) b[i] = uint8_t((i * 37 + i / 100 * 11 + seed) ^ (i >> 8));
  return b;
}

// `strong` = the default bank; only a hinted page-turn cleanup of a plain B/W
// page (no boot/wake, sleep paint or preceding gray screen) runs the quick one.
static const Uc8279X4GcLut* expectedGcBank(bool strong) {
  if (EXPECTED_GC_WAVEFORM == 1) return kUc8279X4GcControl;
  if (EXPECTED_GC_WAVEFORM == 2) return strong ? kUc8279X4GcMergedStrong : kUc8279X4GcMerged;
  return nullptr;
}

static Bytes commandsOf(const EpdBus& bus) {
  Bytes commands;
  for (const auto& w : bus.writes) commands.push_back(w.command);
  return commands;
}

// The stock DU a register GC runs first after PON so the controller holds the
// OTP drive settings rather than its MTP power-on defaults.
static Bytes primeCommands() { return {0x50, 0xe0, 0xe5, 0x03, 0xe1, 0x04, 0x91, 0x90, 0x00, 0x12, 0x92}; }

static Bytes gcCommands(bool powerOn, bool registerGc) {
  Bytes expected = {0x13, 0x10};
  const bool prime = powerOn && registerGc;
  if (prime) {
    const Bytes primeDu = primeCommands();
    expected.insert(expected.end(), primeDu.begin(), primeDu.end());
  }
  for (uint8_t command : {0x50, 0xe0, 0xe5}) expected.push_back(command);
  if (powerOn && !prime) expected.push_back(0x04);
  expected.push_back(0x00);
  if (registerGc)
    for (uint8_t reg = 0x20; reg <= 0x24; ++reg) expected.push_back(reg);
  expected.push_back(0x12);
  expected.push_back(0x10);
  return expected;
}

// Half seeds OLD with the complement of the visible rows and white padding.
static void assertScrubSeed(const Bytes& old, const Bytes& page) {
  assert(old.size() == 60000);
  for (unsigned y = 0; y < 600; ++y)
    for (unsigned x = 0; x < 100; ++x) {
      assert(old[y * 100 + x] == (y >= 120 ? uint8_t(~page[(y - 120) * 100 + x]) : 0xff));
    }
}

// The register GC seeds OLD with the target so only WW/KK rows run; the OTP GC
// keeps the stock white seed (Full/promoted Fast) and complement scrub (Half).
static void assertGcSeed(const EpdBus& bus, const Bytes& page, bool registerGc, RefreshMode mode) {
  const Bytes& old = bus.writes[1].bytes;
  if (registerGc) {
    assert(old == bus.writes[0].bytes);
  } else if (mode == RefreshMode::Half) {
    assertScrubSeed(old, page);
  } else {
    assert(old == Bytes(60000, 0xff));
  }
}

// Full-width B/W drive registers remain the factory values, except the PSR LUT source.
static void checkGcRegisters(const EpdBus& bus, bool registerGc, bool strong) {
  assert(lastRegister(bus, 0x50) == 0x97);
  assert(lastRegister(bus, 0xe0) == 0x02);
  assert(lastRegister(bus, 0xe5) == 0x1e);
  const auto& psr = lastPlane(bus, 0x00);
  assert(psr == Bytes({uint8_t(registerGc ? 0x37 : 0x17), 0x4d}));
  const Uc8279X4GcLut* bank = registerGc ? expectedGcBank(strong) : nullptr;
  unsigned luts = 0;
  for (const auto& w : bus.writes) {
    if (w.command < 0x20 || w.command > 0x24) continue;
    assert(bank != nullptr);
    const auto& row = bank[w.command - 0x20];
    assert(row.reg == w.command);
    assert(w.bytes == Bytes(row.data, row.data + UC8279X4_GC_LUT_LEN));
    if (EXPECTED_GC_WAVEFORM == 2) {
      // Decode the wire row independently: invert, optional idle, then an
      // uninterrupted drive to the final color. No late tail on either rail.
      const uint8_t target = w.command == 0x20 ? 0 : (w.command <= 0x22 ? 2 : 1);
      const uint8_t opposite = target == 0 ? 0 : 3 - target;
      Bytes expected(strong ? 32 : 26, opposite), actual;
      if (strong) expected.insert(expected.end(), 4, 0);
      expected.insert(expected.end(), strong ? 32 : 26, target);
      for (unsigned group = 0; group < 7; ++group) {
        const auto* phase = w.bytes.data() + group * 7;
        if (!phase[0]) continue;
        assert(phase[0] == 1 && phase[5] == 1 && phase[6] == 1);
        for (unsigned i = 1; i <= 4; ++i) actual.insert(actual.end(), phase[i] & 0x3f, phase[i] >> 6);
      }
      assert(actual == expected);
    }
    ++luts;
  }
  assert(luts == (bank != nullptr ? UC8279X4_GC_LUT_COUNT : 0u));
}

static void testGcOnBoard(BoardConfig::Board board) {
  BoardConfig::ACTIVE.board = board;
  BoardConfig::ACTIVE.displayController = BoardConfig::DisplayController::UC8279;
  const auto page = frame(51);
  for (uint8_t variant : {0x02, 0x03, 0x67, 0x68, 0x69, 0xff}) {
    BoardConfig::ACTIVE.displayControllerVariant = variant;
    const bool registerGc = board == BoardConfig::Board::XteinkX4Pro && variant == 0x02 && expectedGcBank(false);
    Uc8279X4Driver driver;
    FreeInkDisplay display(1, 2, 3, 4, 5, 6);
    assert(!display.supportsPageTurnCleanup());
    display._driver = &driver;
    assert(display.supportsPageTurnCleanup() == (registerGc && EXPECTED_GC_WAVEFORM == 2));
    auto& bus = display._bus;
    driver.begin(bus);
    bus.clear();
    // The first paint after begin() is a promoted Fast; it must take the same GC.
    display.preferQuickClean();
    driver.display(bus, page.data(), nullptr, RefreshMode::Fast, false);
    assert(commandsOf(bus) == gcCommands(true, registerGc));
    assert(lastPlane(bus, 0x13).size() == 60000);
    if (registerGc) {
      // The priming DU runs the OTP partial (REG cleared) before the register GC.
      const auto psr = std::find_if(bus.writes.begin(), bus.writes.end(), [](const auto& w) { return w.command == 0x00; });
      assert(psr != bus.writes.end() && psr->bytes == Bytes({0x17, 0x4d}));
    }
    assertGcSeed(bus, page, registerGc, RefreshMode::Fast);
    checkGcRegisters(bus, registerGc, true);
    for (auto mode : {RefreshMode::Full, RefreshMode::Half}) {
      for (bool quick : {false, true}) {
        bus.clear();
        if (quick) display.preferQuickClean();
        driver.display(bus, page.data(), nullptr, mode, false);
        assert(commandsOf(bus) == gcCommands(false, registerGc));
        checkGcRegisters(bus, registerGc, !quick);
        assertGcSeed(bus, page, registerGc, mode);
      }
    }
    // The B/W DU stays on the OTP bank with its stock registers.
    bus.clear();
    driver.display(bus, page.data(), nullptr, RefreshMode::Fast, false);
    assert(commandsOf(bus) == Bytes({0x13, 0x50, 0xe0, 0xe5, 0x03, 0xe1, 0x91, 0x90, 0x00, 0x12, 0x92, 0x10}));
    assert(lastRegister(bus, 0x50) == 0xd7 && lastRegister(bus, 0xe5) == 0x5a);
    assert(lastPlane(bus, 0x00) == Bytes({0x17, 0x4d}));
    bus.clear();
    driver.displayWindow(bus, page.data(), nullptr, 16, 24, 80, 12, false);
    assert(lastPlane(bus, 0x90) == Bytes({0, 16, 0, 95, 0, 144, 0, 155, 1}));
    assert(lastPlane(bus, 0x00) == Bytes({0x17, 0x4d}));
    // Sleep paints power down after the GC completes, then the panel sleeps.
    bus.clear();
    display.preferQuickClean();
    driver.display(bus, page.data(), nullptr, RefreshMode::Half, true);
    Bytes sleepPaint = gcCommands(false, registerGc);
    sleepPaint.push_back(0x02);
    assert(commandsOf(bus) == sleepPaint && !driver._isScreenOn);
    checkGcRegisters(bus, registerGc, true);
    driver.deepSleep(bus);
    assert(bus.writes.back().command == 0x07 && bus.writes.back().bytes == Bytes({0xa5}));
    // A wake GC powers the panel on before the PSR latch and the bank upload.
    driver.begin(bus);
    bus.clear();
    display.preferQuickClean();
    driver.display(bus, page.data(), nullptr, RefreshMode::Full, false);
    assert(commandsOf(bus) == gcCommands(true, registerGc));
    checkGcRegisters(bus, registerGc, true);
    free(driver._grayBase);
  }
}

static void testGcAfterGray(GrayscaleMode mode, bool onePassText) {
  BoardConfig::ACTIVE.board = BoardConfig::Board::XteinkX4Pro;
  BoardConfig::ACTIVE.displayControllerVariant = 0x02;
  FreeInkDisplay display(1, 2, 3, 4, 5, 6);
  Uc8279X4Driver driver;
  display._driver = &driver;
  auto& bus = display._bus;
  const auto page = frame(12), mask = frame(21);
  driver.begin(bus);
  driver.display(bus, page.data(), nullptr, RefreshMode::Full, false);
  driver.setReaderRefresh(onePassText);
  driver.beginGrayscale(bus, page.data(), mode, RefreshMode::Fast, false);
  driver.copyGrayscaleLsb(bus, mask.data());
  driver.copyGrayscaleMsb(bus, mask.data());
  driver.displayGray(bus, page.data(), false, nullptr, false);
  driver.cleanupGrayscaleBuffers(bus, page.data());

  const bool registerGc = expectedGcBank(false) != nullptr;
  for (bool strong : {true, false}) {
    bus.clear();
    display.preferQuickClean();
    driver.display(bus, page.data(), nullptr, RefreshMode::Half, false);
    checkGcRegisters(bus, registerGc, strong);
    // The RC3 text and image packets change analog settings. An OTP DU
    // restores them before the first register GC, even with power still on.
    const auto commands = commandsOf(bus);
    assert(std::count(commands.begin(), commands.end(), 0x12) == (strong && registerGc ? 2 : 1));
    assert(std::count(commands.begin(), commands.end(), 0x04) == 0);
  }
  free(driver._grayBase);
}

void testUc8279X4Gc() {
  const auto saved = BoardConfig::ACTIVE;
  for (auto board : {BoardConfig::Board::XteinkX4Pro, BoardConfig::Board::XteinkX4Classic, BoardConfig::Board::XteinkX4})
    testGcOnBoard(board);
  testGcAfterGray(GrayscaleMode::Overlay, true);
  testGcAfterGray(GrayscaleMode::Absolute, false);
  BoardConfig::ACTIVE = saved;
}
