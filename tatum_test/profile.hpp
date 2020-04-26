#ifndef TATUM_PROFILE
#define TATUM_PROFILE
#include <map>
#include <string>
#include <vector>
#include <memory>

#include "tatum/timing_analyzers.hpp"
#include "tatum/delay_calc/FixedDelayCalculator.hpp"

std::map<std::string,std::vector<double>> profile(size_t num_iterations, std::shared_ptr<tatum::TimingAnalyzer> serial_analyzer);

std::map<std::string,std::vector<double>> profile_rand_incr(size_t num_iterations, float edge_change_prob, std::shared_ptr<tatum::TimingAnalyzer> check_analyzer, std::shared_ptr<tatum::TimingAnalyzer> ref_analyzer, tatum::FixedDelayCalculator& delay_calc, const tatum::TimingGraph& tg);

#endif
