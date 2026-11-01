// Exercise the upstream scalar and AArch64 NEON implementations on identical
// deterministic, nonconstant images. Emulated correctness is not timing data.
#include <fast/fast.h>
#include <svo/direct/patch_score.h>
#include <algorithm>
#include <cstdint>
#include <iostream>
#include <utility>
#include <vector>

static std::vector<std::pair<short, short>> ordered(
    const std::vector<fast::fast_xy>& corners) {
  std::vector<std::pair<short, short>> result;
  for (const auto& corner : corners) result.emplace_back(corner.y, corner.x);
  std::sort(result.begin(), result.end());
  return result;
}

int main() {
  unsigned total = 0;
  for (int width : {32, 64, 128}) {
    for (int padding : {0, 7}) {
      const int height = 48, stride = width + padding;
      std::vector<unsigned char> image(stride * height);
      uint32_t seed = 0x27;
      for (auto& value : image) {
        seed = seed * 1664525u + 1013904223u;
        value = static_cast<unsigned char>(seed >> 24);
      }
      for (short threshold : {10, 20, 40}) {
        std::vector<fast::fast_xy> scalar, neon;
        fast::fast_corner_detect_9(image.data(), width, height, stride, threshold, scalar);
        fast::fast_corner_detect_9_neon(image.data(), width, height, stride, threshold, neon);
        if (ordered(scalar) != ordered(neon)) {
          std::cerr << "FAST scalar/NEON disagreement width=" << width
                    << " stride=" << stride << " threshold=" << threshold << '\n';
          return 1;
        }
        total += scalar.size();
      }
    }
  }
  if (total == 0) return 2;
  // The contiguous overload is scalar; the strided 8x8 overload enters the
  // production NEON implementation on this target. Compare both on the same
  // patches, including padded rows and extreme constant values.
  unsigned patch_cases = 0;
  for (int stride : {8, 11, 32}) {
    for (unsigned trial = 0; trial < 258; ++trial) {
      alignas(16) unsigned char reference[64], compact[64];
      std::vector<unsigned char> strided(stride*8, 0x27);
      uint32_t seed = trial+0x3576;
      for (int i = 0; i < 64; ++i) {
        seed = seed * 1664525u + 1013904223u;
        reference[i] = trial < 2 ? trial*255 : seed >> 24;
        seed = seed * 1664525u + 1013904223u;
        compact[i] = trial < 2 ? (1-trial)*255 : seed >> 24;
        strided[(i/8)*stride+i%8] = compact[i];
      }
      svo::patch_score::ZMSSD<4> score(reference);
      if (score.computeScore(compact) != score.computeScore(strided.data(), stride)) {
        std::cerr << "ZMSSD scalar/NEON disagreement stride=" << stride
                  << " trial=" << trial << '\n';
        return 3;
      }
      ++patch_cases;
    }
  }
  std::cout << "PASS: 18 FAST scalar/NEON cases; total corners=" << total
            << "; ZMSSD scalar/NEON cases=" << patch_cases << '\n';
}
