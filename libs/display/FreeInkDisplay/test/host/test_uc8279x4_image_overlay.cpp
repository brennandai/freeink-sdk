#include <algorithm>
#include <cassert>
#include <vector>

#define private public
#include "FreeInkDisplay.h"
#include "src/driver/Uc8279X4Driver.h"
#undef private
#include "src/lut/Uc8279X4GcLuts.h"
#include "src/lut/X4ProImageLuts.h"

namespace {
using namespace freeink;
using Bytes = std::vector<uint8_t>;
constexpr unsigned kSize = 48000, kPad = 12000;

struct Page {
  Bytes bw = Bytes(kSize, 0xff), low = Bytes(kSize), high = Bytes(kSize);
  explicit Page(bool dense) {
    // Text outside the illustration and an image containing W, B and both
    // mask levels. The large image marks >25% of panel pixels as gray.
    for (unsigned i = 80; i < kSize; i += 100) bw[i] = 0x66;
    for (unsigned i = 1000; i < (dense ? 43000u : 1800u); ++i) {
      bw[i] = 0x03;
      low[i] = 0x30;
      high[i] = 0x3c;
    }
  }
};

const Bytes& reg(const EpdBus& bus, uint8_t cmd) {
  auto it = std::find_if(bus.writes.rbegin(), bus.writes.rend(), [=](const auto& w) { return w.command == cmd; });
  assert(it != bus.writes.rend());
  return it->bytes;
}

unsigned count(const EpdBus& bus, uint8_t cmd) {
  return std::count_if(bus.writes.begin(), bus.writes.end(), [=](const auto& w) { return w.command == cmd; });
}

void assertPlane(const Bytes& upload, const Bytes& expected) {
  assert(upload.size() == kPad + kSize);
  assert(std::all_of(upload.begin(), upload.begin() + kPad, [](uint8_t b) { return b == 0xff; }));
  assert(std::equal(expected.begin(), expected.end(), upload.begin() + kPad));
}

void verifyShortAa(const EpdBus& bus) {
  for (unsigned row = 0; row < 5; ++row) {
    Bytes expected = {1, 2, 2, 1, 1, 1, 1, 0, 0, 0, 0, 0, 1, 1};
    expected.resize(49, 0);
    if (row == 1) expected[3] |= 0x40;
    if (row == 2 || row == 3) expected[2] |= 0x80;
    if (row == 4) expected[3] |= 0x80;
    assert(reg(bus, 0x20 + row) == expected);
  }
  assert(count(bus, 0x12) == 1 && reg(bus, 0x50) == Bytes{0x97});
  for (auto cmd : {0x01, 0x82, 0x2a, 0x30}) assert(count(bus, cmd) == 0);
}

void verifyImagePacket(const EpdBus& bus) {
  constexpr unsigned rows[] = {0, 1, 3, 2, 4};
  for (unsigned i = 0; i < 5; ++i) {
    const auto* start = kX4ProImage + 9 + rows[i] * 49;
    assert(reg(bus, 0x20 + i) == Bytes(start, start + 49));
  }
  assert(count(bus, 0x12) == 1 && reg(bus, 0x82) == Bytes{0x24});
}

void uploadAndShade(FreeInkDisplay& d, const Page& page, bool strips) {
  auto& bus = d._bus;
  bus.clear();
  if (strips) {
    for (auto plane : {FreeInkDisplay::GRAY_PLANE_LSB, FreeInkDisplay::GRAY_PLANE_MSB}) {
      const Bytes& mask = plane == FreeInkDisplay::GRAY_PLANE_LSB ? page.low : page.high;
      for (unsigned y = 0; y < 480; y += 80) d.writeGrayscalePlaneStrip(plane, mask.data() + y * 100, y, 80);
    }
  } else {
    // Snapshot fallback reuses the live framebuffer for each mask. The driver
    // must have kept its own base; the application restores its snapshot later.
    d.setFramebuffer(page.low.data());
    d.copyGrayscaleLsbBuffers(d.getFrameBuffer());
    d.setFramebuffer(page.high.data());
    d.copyGrayscaleMsbBuffers(d.getFrameBuffer());
  }
  assert(count(bus, 0x12) == 0);
  const auto& low = reg(bus, 0x10);
  const auto& high = reg(bus, 0x13);
  for (unsigned i = 0; i < kSize; ++i) {
    const uint8_t folded = page.bw[i] | page.low[i];
    assert(low[kPad + i] == uint8_t(~folded));
    assert(high[kPad + i] == uint8_t(~(folded ^ page.high[i])));
  }
  bus.clear();
  d.displayGrayBuffer();
  verifyShortAa(bus);
  assertPlane(reg(bus, 0x10), page.bw);
  assertPlane(reg(bus, 0x13), page.bw);
  d.cleanupGrayscaleBuffers(page.bw.data());
}

void testReaderPass(FreeInkDisplay& d) {
  auto& bus = d._bus;
  for (bool dense : {false, true}) {
    const Page page(dense);
    d.clearScreen(0xff);
    // A clean preview with an empty image box may still be finishing while
    // the caller composes the actual illustration into its write buffer.
    d.displayBufferAsync(FreeInkDisplay::HALF_REFRESH);
    d.setFramebuffer(page.bw.data());
    bus.clear();
    d.preferImageOverlay();
    d.displayGrayscaleBase(FreeInkDisplay::FAST_REFRESH);
    assert(!d.isRefreshPending());
    assert(count(bus, 0x12) == 1 && reg(bus, 0xe5) == Bytes{0x5a});
    // First base uses OTP DU; later pages can use stock PRE_BW_MID. Neither
    // re-runs the GC. The B/W image (not the preview) reaches both RAM planes.
    if (dense) assert(reg(bus, 0x20).size() == 42);
    else assert(count(bus, 0x20) == 0);
    assertPlane(reg(bus, 0x10), page.bw);
    assertPlane(reg(bus, 0x13), page.bw);
    uploadAndShade(d, page, dense);
    bus.clear();
    d.preferQuickClean();
    d.displayBuffer(FreeInkDisplay::HALF_REFRESH);
    assert(count(bus, 0x12) == 1);
    if (EXPECTED_GC_WAVEFORM == 2) {
      const auto& row = kUc8279X4GcMergedStrong[1];
      assert(reg(bus, row.reg) == Bytes(row.data, row.data + UC8279X4_GC_LUT_LEN));
    }
  }
}

void testReplacement(FreeInkDisplay& d) {
  const Page page(true);
  auto& bus = d._bus;
  for (unsigned interrupted = 0; interrupted < 7; ++interrupted) {
    d.setFramebuffer(page.bw.data());
    d.preferImageOverlay();
    if (interrupted == 0) {
      d.displayBuffer(FreeInkDisplay::FAST_REFRESH);  // competing refresh before the hinted base
    } else if (interrupted == 1) {
      d.requestResync();
    } else if (interrupted == 2) {
      // A mode-bound Direct image wins even if a caller hinted just before it.
      assert(d.displayGrayscaleBase(GrayscaleMode::Direct));
      d.copyGrayscaleBuffers(page.low.data(), page.high.data());
      bus.clear();
      d.displayGrayBuffer();
      verifyImagePacket(bus);
    } else {
      d.displayGrayscaleBase(FreeInkDisplay::FAST_REFRESH);
      d.copyGrayscaleLsbBuffers(page.low.data());
      bus.clear();
      if (interrupted == 3) d.displayGrayBuffer();  // missing MSB
      if (interrupted == 4) {
        d.copyGrayscaleMsbBuffers(nullptr);
        d.copyGrayscaleMsbBuffers(page.high.data());
        d.displayGrayBuffer();  // malformed upload stays failed
      }
      if (interrupted == 6) d.setReaderRefresh(false);  // abandon the image and enter the UI
      d.cleanupGrayscaleBuffers(page.bw.data());  // also covers an abandoned pass
      assert(count(bus, 0x12) == 0);
    }
    d.displayGrayscaleBase(FreeInkDisplay::FAST_REFRESH);
    d.copyGrayscaleBuffers(page.low.data(), page.high.data());
    bus.clear();
    d.displayGrayBuffer();
    verifyImagePacket(bus);  // next ordinary dense overlay keeps RC3 quality
  }
}

void testAfterReaderText(FreeInkDisplay& d) {
  const Page page(true);
  auto& bus = d._bus;
  d.setReaderRefresh(true);
  d.displayGrayscaleBase(FreeInkDisplay::FAST_REFRESH);
  d.copyGrayscaleBuffers(page.low.data(), page.high.data());
  d.displayGrayBuffer();
  d.cleanupGrayscaleBuffers(page.bw.data());
  // The text packet programmed analog settings. Even with reader mode still
  // enabled, the next opted-in base must be a separate OTP B/W activation.
  bus.clear();
  d.preferImageOverlay();
  d.displayGrayscaleBase(FreeInkDisplay::FAST_REFRESH);
  assert(count(bus, 0x12) == 1 && reg(bus, 0x00) == Bytes({0x17, 0x4d}));
  assert(count(bus, 0x20) == 0 && reg(bus, 0xe5) == Bytes{0x5a});
  assertPlane(reg(bus, 0x13), page.bw);
  uploadAndShade(d, page, false);
  d.setReaderRefresh(false);
}

void testAfterDirectImage(FreeInkDisplay& d) {
  const Page page(false);
  auto& bus = d._bus;
  assert(d.displayGrayscaleBase(GrayscaleMode::Direct));
  d.copyGrayscaleBuffers(page.low.data(), page.high.data());
  d.displayGrayBuffer();
  d.cleanupGrayscaleBuffers(page.bw.data());
  bus.clear();
  d.preferImageOverlay();
  d.displayGrayscaleBase(FreeInkDisplay::FAST_REFRESH);
  // Direct left an untracked full-quality image on the glass. Its mandatory
  // GC must not be skipped, or preceded by a flash of the destination.
  assert(count(bus, 0x12) == 2);
  if (EXPECTED_GC_WAVEFORM == 2) {
    assert(bus.writes[0].bytes == Bytes(kPad + kSize, 0xff));
    assert(bus.writes[1].bytes == bus.writes[0].bytes);
    const auto& row = kUc8279X4GcMergedStrong[1];
    assert(reg(bus, row.reg) == Bytes(row.data, row.data + UC8279X4_GC_LUT_LEN));
  }
  assertPlane(reg(bus, 0x13), page.bw);
  uploadAndShade(d, page, false);
}
}  // namespace

void testUc8279X4ImageOverlay() {
  const auto saved = BoardConfig::ACTIVE;
  BoardConfig::ACTIVE.board = BoardConfig::Board::XteinkX4Pro;
  BoardConfig::ACTIVE.displayController = BoardConfig::DisplayController::UC8279;
  BoardConfig::ACTIVE.displayControllerVariant = 2;
  Uc8279X4Driver driver;
  FreeInkDisplay d(1, 2, 3, 4, 5, 6);
  d._driver = &driver;
  d.begin();
#if FREEINK_DEVICE_X4PRO
  testReaderPass(d);
  testReplacement(d);
  testAfterReaderText(d);
  testAfterDirectImage(d);
  // Allocation failure: show the B/W image; raw masks cannot represent the
  // missing distinction between unchanged white and black.
  free(driver._grayBase);
  driver._grayBase = nullptr;
  const Page page(true);
  d.preferImageOverlay();
  d.displayGrayscaleBase(FreeInkDisplay::FAST_REFRESH);
  d.copyGrayscaleBuffers(page.low.data(), page.high.data());
  d._bus.clear();
  d.displayGrayBuffer();
  assert(count(d._bus, 0x12) == 0);
  d.setInverted(true);
  assert(!d.grayscaleCapabilities(GrayscaleMode::ImageOverlay).supported());
  d.setInverted(false);
  BoardConfig::ACTIVE.board = BoardConfig::Board::XteinkX4Classic;
  assert(!d.grayscaleCapabilities(GrayscaleMode::ImageOverlay).supported());
  BoardConfig::ACTIVE.board = BoardConfig::Board::XteinkX4Pro;
  BoardConfig::ACTIVE.displayControllerVariant = 3;
#endif
  assert(!d.grayscaleCapabilities(GrayscaleMode::ImageOverlay).supported());
  d.releaseBuffers();
  free(driver._grayBase);
  BoardConfig::ACTIVE = saved;
}
