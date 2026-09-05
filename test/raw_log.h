/**
 * @file raw_log.h
 * @brief Parser for recordings made with tools/record_raw.py (frames written
 *        by MainCore built with -DRAW_LOG_OUTPUT).
 */
#ifndef RAW_LOG_H
#define RAW_LOG_H

#include <cstdio>
#include <cstring>
#include <vector>
#include "shared_types.h"
#include "localizer/localizer.h"

struct Frame
{
  uint8_t type;
  std::vector<uint8_t> payload;
};

static inline size_t frameSize(uint8_t type)
{
  switch (type)
  {
  case RAW_LOG_TYPE_IMU: return sizeof(ImuRawData_t);
  case RAW_LOG_TYPE_POSITION: return sizeof(PositionData_t);
  case RAW_LOG_TYPE_STATS: return sizeof(ProcessingStats_t);
  case RAW_LOG_TYPE_PRESET: return sizeof(AccelBiasPreset_t);
  default: return 0;
  }
}

/** Extract all valid frames; `bad` receives the number of skipped bytes. */
static inline std::vector<Frame> parse(const std::vector<uint8_t> &d, size_t &bad)
{
  std::vector<Frame> out;
  size_t i = 0;
  bad = 0;
  while (i + 4 <= d.size())
  {
    const size_t size = (d[i] == RAW_LOG_SYNC0 && d[i + 1] == RAW_LOG_SYNC1) ? frameSize(d[i + 2]) : 0;
    if (size == 0)
    {
      i++; bad++;
      continue;
    }
    if (i + 4 + size > d.size()) break;
    uint8_t chk = d[i + 2];
    for (size_t k = 0; k < size; k++) chk ^= d[i + 3 + k];
    if (chk != d[i + 3 + size])
    {
      i++; bad++;
      continue;
    }
    Frame f;
    f.type = d[i + 2];
    f.payload.assign(d.begin() + i + 3, d.begin() + i + 3 + size);
    out.push_back(f);
    i += 4 + size;
  }
  return out;
}

/** Read a whole file; returns false if it cannot be opened. */
static inline bool readFile(const char *path, std::vector<uint8_t> &data)
{
  FILE *f = std::fopen(path, "rb");
  if (!f) return false;
  uint8_t buf[65536];
  size_t n;
  while ((n = std::fread(buf, 1, sizeof(buf), f)) > 0) data.insert(data.end(), buf, buf + n);
  std::fclose(f);
  return true;
}

#endif // RAW_LOG_H
