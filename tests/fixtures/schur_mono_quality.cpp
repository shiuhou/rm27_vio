// Exercise the linked production quality rules across the monocular handoff.
#include "schur_mono_fixture.h"

int main(int argc, char** argv) {
  if (argc != 2) return 2;
  MonoProbe mono(svo::CameraBundle::loadFromYaml(argv[1]));
  mono.setPreviousImagePopulation(180);
  require(!mono.evaluateQuality(105), "premetric population-drop rule changed");
  require(mono.metricObservationHistory() == 0, "premetric history contaminated");
  mono.beginMetricEpoch();
  mono.setPreviousImagePopulation(180);
  require(mono.clonesPending(), "real mono warmup state not reached");
  require(!mono.evaluateQuality(105), "warmup population-drop rule changed");
  require(mono.metricObservationHistory() == 0, "warmup history contaminated");
  mono.matureMetricWindow();
  mono.setPreviousImagePopulation(180);
  require(!mono.evaluateBaseQuality(105), "original base quality rule changed");
  require(mono.evaluateQuality(105), "mature count compared with image population");
  require(mono.metricObservationHistory() == 105, "mature count was not recorded");
  require(mono.evaluateQuality(65), "drop equal to 40 must remain accepted");
  require(mono.evaluateQuality(105), "mature count increase rejected");
  require(!mono.evaluateQuality(60), "mature drop of 45 must remain rejected");
  mono.setPreviousImagePopulation(180, true);
  require(mono.evaluateQuality(105), "keyframe count increase rejected");
  require(mono.evaluateQuality(60), "previous keyframe drop exemption changed");
  require(!mono.evaluateQuality(39), "keyframe exemption bypassed absolute minimum");
  mono.resetEpoch();
  require(mono.metricObservationHistory() == 0, "reset retained mature history");
  mono.beginMetricEpoch();
  mono.matureMetricWindow();
  mono.setPreviousImagePopulation(180);
  require(mono.evaluateQuality(105), "new epoch used stale mature history");
  std::cout << "PASS production mono quality and epoch boundaries\n";
  return 0;
}
