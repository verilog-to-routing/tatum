#include <iostream>
#include <cmath>
#include <chrono>
#include <random>
#include "profile.hpp"

#include "tatum/TimingGraph.hpp"

#ifdef TATUM_STA_PROFILE_VTUNE
#include "ittnotify.h"
#endif

#ifdef TATUM_STA_PROFILE_CALLGRIND
#include <valgrind/callgrind.h>
#endif


typedef std::chrono::duration<double> dsec;
typedef std::chrono::high_resolution_clock Clock;

std::map<std::string,std::vector<double>> profile(size_t num_iterations, std::shared_ptr<tatum::TimingAnalyzer> serial_analyzer) {
    //To selectively profile using callgrind:
    //  valgrind --tool=callgrind --collect-atstart=no --instr-atstart=no --cache-sim=yes --cacheuse=yes ./command

    std::map<std::string,std::vector<double>> prof_data;

#ifdef TATUM_STA_PROFILE_CALLGRIND
    CALLGRIND_START_INSTRUMENTATION;
#endif

    for(size_t i = 0; i < num_iterations; i++) {
        //Analyze

#ifdef TATUM_STA_PROFILE_VTUNE
        __itt_resume();
#endif

#ifdef TATUM_STA_PROFILE_CALLGRIND
        CALLGRIND_TOGGLE_COLLECT;
#endif

        serial_analyzer->update_timing();

#ifdef TATUM_STA_PROFILE_CALLGRIND
        CALLGRIND_TOGGLE_COLLECT;
#endif

#ifdef TATUM_STA_PROFILE_VTUNE
        __itt_pause();
#endif

        for(auto key : {"arrival_pre_traversal_sec", "arrival_traversal_sec", "required_pre_traversal_sec", "required_traversal_sec", "reset_sec", "update_slack_sec", "analysis_sec"}) {
            prof_data[key].push_back(serial_analyzer->get_profiling_data(key));
        }


        std::cout << ".";
        std::cout.flush();
    }

#ifdef TATUM_STA_PROFILE_CALLGRIND
    CALLGRIND_STOP_INSTRUMENTATION;
#endif

    return prof_data;
}

std::map<std::string,std::vector<double>> profile_rand_incr(size_t num_iterations, std::shared_ptr<tatum::TimingAnalyzer> serial_analyzer, tatum::FixedDelayCalculator& delay_calc, const tatum::TimingGraph& tg) {

    std::map<std::string,std::vector<double>> prof_data;

    std::minstd_rand rng;
    std::uniform_int_distribution<size_t> uniform_distr(0, tg.edges().size() - 1);
    std::normal_distribution<float> normal_distr(0, 1e-9);

    for(size_t i = 0; i < num_iterations; i++) {

        if (i > 0) {
            //Randomly invalidate edges

            size_t EDGES_TO_INVALIDATE = 0.001 * tg.edges().size();
            for (size_t j = 0; j < EDGES_TO_INVALIDATE; j++) {
                size_t iedge = uniform_distr(rng);
                tatum::EdgeId edge(iedge);

                //Invalidate
                serial_analyzer->invalidate_edge(edge);

                //Set new delays
                if (tg.edge_type(edge) == tatum::EdgeType::PRIMITIVE_CLOCK_CAPTURE) {
                    float new_setup = std::max<float>(0, delay_calc.setup_time(tg, edge).value() + normal_distr(rng));
                    float new_hold = std::max<float>(0, delay_calc.hold_time(tg, edge).value() + normal_distr(rng));
                    delay_calc.set_setup_time(tg, edge, tatum::Time(new_setup));
                    delay_calc.set_hold_time(tg, edge, tatum::Time(new_hold));
                } else {
                    float new_max = std::max<float>(0, delay_calc.max_edge_delay(tg, edge).value() + normal_distr(rng));
                    float new_min = std::max<float>(0, delay_calc.min_edge_delay(tg, edge).value() + normal_distr(rng));
                    delay_calc.set_max_edge_delay(tg, edge, tatum::Time(new_max));
                    delay_calc.set_min_edge_delay(tg, edge, tatum::Time(new_min));
                }
            }
        }

        //Analyze
        serial_analyzer->update_timing();


        for(auto key : {"arrival_pre_traversal_sec", "arrival_traversal_sec", "required_pre_traversal_sec", "required_traversal_sec", "reset_sec", "update_slack_sec", "analysis_sec"}) {
            prof_data[key].push_back(serial_analyzer->get_profiling_data(key));
        }


        std::cout << ".";
        std::cout.flush();
    }

    return prof_data;
}
