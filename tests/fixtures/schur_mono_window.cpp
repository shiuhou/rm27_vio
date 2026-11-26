// Window mechanics only. The five-landmark case is below VIO quality minimum.
#include "schur_mono_fixture.h"

class WindowProbe : public MonoProbe {
 public:
  using MonoProbe::MonoProbe;
  int step(int index, size_t count, bool keyframe) {
    auto bundle = makeFrame(index);
    bundle->at(0)->num_features_ = count;
    schur_vins_->Forward(bundle);
    const int valid = schur_vins_->Backward(bundle);
    if (keyframe) bundle->setKeyframe();
    // Production Base order: finish frontend before managing the clone window.
    schur_vins_->SetKeyframe(bundle->isKeyframe());
    last_frames_ = bundle;
    const auto point = bundle->at(0)->landmark_vec_.at(0);
    require(point && point->local_obs_.size() == size_t(std::min(index + 1, 4)),
            "steady landmark lost observations during management");
    require(bundle->get_T_W_B().getPosition().allFinite(), "nonfinite corrected pose");
    if (index >= 2) require(!clonesPending(), "mature mono returned to clone warmup");
    return valid;
  }
};

int main(int argc, char** argv) {
  if (argc != 2) return 2;
  const auto cameras = svo::CameraBundle::loadFromYaml(argv[1]);
  for (int policy = 0; policy < 3; ++policy) {
    WindowProbe mono(cameras);
    mono.beginMetricEpoch();
    // 100 observations exercise second-newest removal; <10 selects oldest.
    const int count = policy == 2 ? 5 : 100;
    for (int frame = 1; frame <= 20; ++frame) {
      const int valid = mono.step(frame, count, policy == 1);
      if (frame >= 2) require(valid == count, "steady mature observation count changed");
    }
  }
  std::cout << "PASS production mono window: 60 frames across both removal policies\n";
  return 0;
}
