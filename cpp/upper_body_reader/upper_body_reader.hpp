#pragma once

#include <functional>

/**
 * Abstract upper-body recorder. Implementations:
 *   - ExoUpperBodyReader: records raw exo encoder ticks (no robot).
 *   - G1UpperBodyReader:  drives the G1 from exo readings and records
 *                         measured/commanded G1 joint angles in radians.
 */
class UpperBodyReader {
 public:
  virtual ~UpperBodyReader() = default;

  /** Drive recording until `stop()` returns true. While `pause()` returns
   *  true, samples are still consumed but not written to disk. */
  virtual void collect_loop(const std::function<int()>&  collection_id,
                            const std::function<bool()>& stop,
                            const std::function<bool()>& pause) = 0;
};
