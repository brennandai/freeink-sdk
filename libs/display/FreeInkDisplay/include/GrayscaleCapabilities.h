#pragma once

#include <stdint.h>

namespace freeink {

// Overlay uses the existing base + LSB/MSB mask API. Absolute requires planes
// containing every pixel, including the B/W background. Direct uses that same
// encoding with one combined activation using the full-quality image waveform.
// ImageOverlay uses complete overlay masks over a separately painted B/W image
// and keeps the short AA waveform regardless of coverage. Querying selects no mode.
enum class GrayscaleMode : uint8_t { Overlay, Absolute, Direct, ImageOverlay };
enum class GrayscaleEncoding : uint8_t { Unsupported, OverlayMasks, AbsolutePlanes };
enum class GrayscaleBase : uint8_t { Separate, Combined };

struct GrayscaleCapabilities {
  // Overlay masks: black/white=00 (distinguished by the B/W base), dark=11,
  // light=01, in (LSB, MSB) order. Absolute planes: black=00, dark=10,
  // light=01, white=11. Encoding does not promise independence from prior ink.
  GrayscaleEncoding encoding = GrayscaleEncoding::Unsupported;
  GrayscaleBase base = GrayscaleBase::Separate;
  bool stripUploads = false;
  // An ordinary asynchronous B/W refresh is a valid base for this mode.
  bool asyncBase = false;
  // Plane staging touches no SPI and is safe during a pending B/W waveform.
  bool stagingWhileBusy = false;

  constexpr bool supported() const { return encoding != GrayscaleEncoding::Unsupported; }
};

}  // namespace freeink
