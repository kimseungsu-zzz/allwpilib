// Copyright (c) 2026 WPILib contributors.

#pragma once

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <mutex>
#include <string>
#include <unordered_map>

#include "studica/Colore.h"

namespace studica::detail {

struct ColorReference {
  float x = 0.0F;
  float y = 0.0F;
  float threshold = 0.05F;
};

class ColorMatcher final {
 public:
  bool SetReference(const std::string& name, float x, float y,
                    float threshold) noexcept {
    if (name.empty() || name.size() >= STUDICA_COLORE_LABEL_CAPACITY ||
        !std::isfinite(x) || !std::isfinite(y) || !std::isfinite(threshold) ||
        threshold <= 0.0F) {
      return false;
    }
    std::scoped_lock lock{m_mutex};
    m_references[name] = {x, y, threshold};
    return true;
  }

  bool GetReference(const std::string& name,
                    StudicaColoreMatchResult* result) const noexcept {
    if (!result) return false;
    std::scoped_lock lock{m_mutex};
    auto it = m_references.find(name);
    if (it == m_references.end()) return false;
    *result = {};
    result->structSize = sizeof(*result);
    std::strncpy(result->label, name.c_str(),
                 STUDICA_COLORE_LABEL_CAPACITY - 1);
    result->label[STUDICA_COLORE_LABEL_CAPACITY - 1] = '\0';
    result->measuredX = it->second.x;
    result->measuredY = it->second.y;
    result->confidence = 1.0F;
    result->valid = 1;
    return true;
  }

  bool Match(float x, float y, StudicaColoreMatchResult* result) const noexcept {
    if (!result || !std::isfinite(x) || !std::isfinite(y)) return false;
    std::scoped_lock lock{m_mutex};
    *result = {};
    result->structSize = sizeof(*result);
    result->measuredX = x;
    result->measuredY = y;
    float bestDistance = std::numeric_limits<float>::infinity();
    const ColorReference* best = nullptr;
    const std::string* bestName = nullptr;
    for (const auto& [name, reference] : m_references) {
      const float dx = x - reference.x;
      const float dy = y - reference.y;
      const float distance = std::sqrt(dx * dx + dy * dy);
      if (distance < bestDistance) {
        bestDistance = distance;
        best = &reference;
        bestName = &name;
      }
    }
    if (!best || bestDistance > best->threshold) return true;
    std::strncpy(result->label, bestName->c_str(),
                 STUDICA_COLORE_LABEL_CAPACITY - 1);
    result->label[STUDICA_COLORE_LABEL_CAPACITY - 1] = '\0';
    result->confidence =
        std::clamp(1.0F - bestDistance / best->threshold, 0.0F, 1.0F);
    result->valid = 1;
    return true;
  }

 private:
  mutable std::mutex m_mutex;
  std::unordered_map<std::string, ColorReference> m_references;
};

inline void FillMatchResult(StudicaColoreMatchResult* result, float x,
                            float y) noexcept {
  if (!result) return;
  *result = {};
  result->structSize = sizeof(*result);
  result->measuredX = x;
  result->measuredY = y;
}

}  // namespace studica::detail
