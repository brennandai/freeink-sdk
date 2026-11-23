#pragma once

#include <stdint.h>

// UC8279 X4 Pro B/W GC banks for the register-LUT path (PSR REG=1), derived
// from the MTP of a LUT_VER 0x02 (QY) panel. Rows are register-prefixed like
// the driver's AA banks. Under CDI 0x97 (DDX=01) a pixel selects its row from
// (OLD=DTM1, NEW=DTM2), 1 = white: WW 0x21 = (1,1), KW 0x22 = (0,1),
// WK 0x23 = (1,0), KK 0x24 = (0,0). Each row holds 7 groups of [group repeat,
// four phases of (rail << 6 | frames), state-1 repeat, state-2 repeat]; rail 1
// drives toward black and rail 2 toward white. Unused groups stay zero.

namespace freeink {

constexpr uint8_t UC8279X4_GC_LUT_COUNT = 5;
constexpr uint8_t UC8279X4_GC_LUT_LEN = 49;

struct Uc8279X4GcLut {
  uint8_t reg;
  uint8_t data[UC8279X4_GC_LUT_LEN];
};

// Byte-exact copy of the panel's OTP GC bank for the forced 30 °C range
// (temperature range 4, MTP 0x4C0..0x5B4), the bank the OTP path selects with
// TSSET 0x1E. One-time validation build: it must look identical to the OTP GC,
// including the late extra black push.
constexpr Uc8279X4GcLut kUc8279X4GcControl[UC8279X4_GC_LUT_COUNT] = {
    {0x20, {0x01, 0x20, 0x04, 0x0E, 0x0B, 0x01, 0x01, 0x01, 0x12, 0x07, 0x00, 0x00, 0x01, 0x01}},  // VCOM
    {0x21, {0x01, 0x60, 0x04, 0x8E, 0x8B, 0x01, 0x01, 0x01, 0x12, 0x87, 0x00, 0x00, 0x01, 0x01}},  // WW: final white
    {0x22, {0x01, 0x60, 0x04, 0x8E, 0x8B, 0x01, 0x01, 0x01, 0x12, 0x87, 0x00, 0x00, 0x01, 0x01}},  // KW: final white
    {0x23, {0x01, 0xA0, 0x04, 0x4E, 0x0B, 0x01, 0x01, 0x01, 0x52, 0x07, 0x00, 0x00, 0x01, 0x01}},  // WK: final black
    {0x24, {0x01, 0xA0, 0x04, 0x4E, 0x0B, 0x01, 0x01, 0x01, 0x52, 0x07, 0x00, 0x00, 0x01, 0x01}},  // KK: final black
};

// Merged payback: invert (26 frames), then one continuous 26-frame push to
// the target that ends the waveform on every row (52 frames, no rest). Each
// pixel receives 26 frames on each rail.
constexpr Uc8279X4GcLut kUc8279X4GcMerged[UC8279X4_GC_LUT_COUNT] = {
    {0x20, {0x01, 0x0D, 0x0D, 0x0D, 0x0D, 0x01, 0x01}},  // VCOM
    {0x21, {0x01, 0x4D, 0x4D, 0x8D, 0x8D, 0x01, 0x01}},  // WW: final white
    {0x22, {0x01, 0x4D, 0x4D, 0x8D, 0x8D, 0x01, 0x01}},  // KW: final white
    {0x23, {0x01, 0x8D, 0x8D, 0x4D, 0x4D, 0x01, 0x01}},  // WK: final black
    {0x24, {0x01, 0x8D, 0x8D, 0x4D, 0x4D, 0x01, 0x01}},  // KK: final black
};

// Strong merged payback for leaving dark or grayscale screens (wake, sleep,
// images): the OTP GC's own 32 frames per rail, as invert (32 frames), rest
// (4), then one continuous 32-frame push (68 frames).
constexpr Uc8279X4GcLut kUc8279X4GcMergedStrong[UC8279X4_GC_LUT_COUNT] = {
    {0x20, {0x01, 0x20, 0x04, 0x10, 0x10, 0x01, 0x01}},  // VCOM
    {0x21, {0x01, 0x60, 0x04, 0x90, 0x90, 0x01, 0x01}},  // WW: final white
    {0x22, {0x01, 0x60, 0x04, 0x90, 0x90, 0x01, 0x01}},  // KW: final white
    {0x23, {0x01, 0xA0, 0x04, 0x50, 0x50, 0x01, 0x01}},  // WK: final black
    {0x24, {0x01, 0xA0, 0x04, 0x50, 0x50, 0x01, 0x01}},  // KK: final black
};

namespace uc8279x4gc {

constexpr uint8_t GROUP_LEN = 7;
constexpr uint8_t GROUP_COUNT = UC8279X4_GC_LUT_LEN / GROUP_LEN;
constexpr uint8_t PHASES_PER_GROUP = 4;
constexpr uint8_t RAIL_IDLE = 0;
constexpr uint8_t RAIL_BLACK = 1;
constexpr uint8_t RAIL_WHITE = 2;

constexpr uint8_t railOf(uint8_t phase) { return phase >> 6; }
constexpr uint8_t framesOf(uint8_t phase) { return phase & 0x3F; }
constexpr uint8_t phaseAt(const Uc8279X4GcLut& lut, uint8_t group, uint8_t phase) {
  return lut.data[group * GROUP_LEN + 1 + phase];
}

// Every group is either all zero or runs exactly once (group and both state
// repeats = 1), so summed phase frames are the exact frames the IC drives.
constexpr bool groupsRunOnce(const Uc8279X4GcLut& lut) {
  for (uint8_t g = 0; g < GROUP_COUNT; g++) {
    const uint8_t* group = &lut.data[g * GROUP_LEN];
    bool zero = true;
    for (uint8_t i = 0; i < GROUP_LEN; i++) zero = zero && group[i] == 0;
    if (!zero && (group[0] != 1 || group[5] != 1 || group[6] != 1)) return false;
  }
  return true;
}

constexpr uint16_t railFrames(const Uc8279X4GcLut& lut, uint8_t rail) {
  uint16_t total = 0;
  for (uint8_t g = 0; g < GROUP_COUNT; g++)
    for (uint8_t p = 0; p < PHASES_PER_GROUP; p++)
      if (railOf(phaseAt(lut, g, p)) == rail) total += framesOf(phaseAt(lut, g, p));
  return total;
}

// Same frame count as VCOM in every phase slot, no VDHR rail, equal VSH/VSL time.
constexpr bool rowIsBalanced(const Uc8279X4GcLut& lut, const Uc8279X4GcLut& vcom) {
  for (uint8_t g = 0; g < GROUP_COUNT; g++)
    for (uint8_t p = 0; p < PHASES_PER_GROUP; p++)
      if (framesOf(phaseAt(lut, g, p)) != framesOf(phaseAt(vcom, g, p)) || railOf(phaseAt(lut, g, p)) == 3)
        return false;
  return groupsRunOnce(lut) && railFrames(lut, RAIL_BLACK) == railFrames(lut, RAIL_WHITE);
}

// The payback is one continuous push that ends the waveform: after the last
// idle phase only one rail drives, for the full opposing-rail time.
constexpr bool endsWithSinglePush(const Uc8279X4GcLut& lut) {
  uint8_t tailRail = RAIL_IDLE;
  uint16_t tailFrames = 0;
  for (uint8_t g = 0; g < GROUP_COUNT; g++)
    for (uint8_t p = 0; p < PHASES_PER_GROUP; p++) {
      const uint8_t phase = phaseAt(lut, g, p);
      if (framesOf(phase) == 0) continue;
      if (railOf(phase) == RAIL_IDLE || railOf(phase) != tailRail) {
        tailRail = railOf(phase);
        tailFrames = 0;
      }
      if (tailRail != RAIL_IDLE) tailFrames += framesOf(phase);
    }
  const uint8_t opposite = tailRail == RAIL_BLACK ? RAIL_WHITE : RAIL_BLACK;
  return tailRail != RAIL_IDLE && tailFrames == railFrames(lut, opposite);
}

constexpr bool bankIsSafe(const Uc8279X4GcLut (&bank)[UC8279X4_GC_LUT_COUNT]) {
  if (!groupsRunOnce(bank[0]) || railFrames(bank[0], RAIL_BLACK) != 0 || railFrames(bank[0], RAIL_WHITE) != 0 ||
      railFrames(bank[0], 3) != 0)
    return false;
  for (uint8_t i = 0; i < UC8279X4_GC_LUT_COUNT; i++)
    if (bank[i].reg != 0x20 + i || (i > 0 && !rowIsBalanced(bank[i], bank[0]))) return false;
  return true;
}

constexpr bool bankEndsWithSinglePush(const Uc8279X4GcLut (&bank)[UC8279X4_GC_LUT_COUNT]) {
  for (uint8_t i = 1; i < UC8279X4_GC_LUT_COUNT; i++)
    if (!endsWithSinglePush(bank[i])) return false;
  return true;
}

}  // namespace uc8279x4gc

static_assert(uc8279x4gc::bankIsSafe(kUc8279X4GcControl), "control GC bank must be DC balanced and well formed");
static_assert(uc8279x4gc::bankIsSafe(kUc8279X4GcMerged), "merged GC bank must be DC balanced and well formed");
static_assert(!uc8279x4gc::bankEndsWithSinglePush(kUc8279X4GcControl), "control keeps the factory staged ending");
static_assert(uc8279x4gc::bankEndsWithSinglePush(kUc8279X4GcMerged), "merged GC must end with one continuous push");
static_assert(uc8279x4gc::bankIsSafe(kUc8279X4GcMergedStrong), "strong GC bank must be DC balanced and well formed");
static_assert(uc8279x4gc::bankEndsWithSinglePush(kUc8279X4GcMergedStrong), "strong GC must end with one continuous push");

}  // namespace freeink
