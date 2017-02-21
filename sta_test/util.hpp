#pragma once
#include <set>
#include <vector>

#include "TimingGraphFwd.hpp"
#include "TimingConstraintsFwd.hpp"
#include "FixedDelayCalculator.hpp"

float relative_error(float A, float B);

void remap_delay_calculator(const tatum::TimingGraph& tg, tatum::FixedDelayCalculator& dc, const tatum::util::linear_map<tatum::EdgeId,tatum::EdgeId>& edge_id_map);
