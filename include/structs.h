#pragma once

#include <cstdint>

struct particle{
  float x, y;
  float theta;
  float score[5];
  // 다중 가설 초기화에서 어느 가설에서 태어났는지의 라벨입니다.
  // 리샘플링/전파를 구조체 복사로 그대로 통과하며, 모드별 질량 집계에 씁니다.
  uint16_t mode{0};
};
