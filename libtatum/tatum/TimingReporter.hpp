#ifndef TATUM_TIMING_REPORTER_HPP
#define TATUM_TIMING_REPORTER_HPP
#include <iosfwd>
#include "tatum/TimingGraphFwd.hpp"
#include "tatum/TimingConstraintsFwd.hpp"
#include "tatum/timing_analyzers.hpp"

#include "tatum/TimingGraphNameResolver.hpp"
#include "tatum/report/TimingPath.hpp"
#include "tatum/report/TimingPathCollector.hpp"
#include "tatum/report/TimingReportTagRetriever.hpp"

namespace tatum {

constexpr size_t REPORT_TIMING_DEFAULT_NPATHS=100;

//A class for generating timing reports
class TimingReporter {
    public:
        TimingReporter(const TimingGraphNameResolver& name_resolver,
                       const tatum::TimingGraph& timing_graph, 
                       const tatum::TimingConstraints& timing_constraints, 
                       float unit_scale=1e-9,
                       size_t precision=3);
    public:
        void report_timing_setup(std::string filename, const tatum::SetupTimingAnalyzer& setup_analyzer, size_t npaths=REPORT_TIMING_DEFAULT_NPATHS) const;
        void report_timing_setup(std::ostream& os, const tatum::SetupTimingAnalyzer& setup_analyzer, size_t npaths=REPORT_TIMING_DEFAULT_NPATHS) const;

        void report_timing_hold(std::string filename, const tatum::HoldTimingAnalyzer& hold_analyzer, size_t npaths=REPORT_TIMING_DEFAULT_NPATHS) const;
        void report_timing_hold(std::ostream& os, const tatum::HoldTimingAnalyzer& hold_analyzer, size_t npaths=REPORT_TIMING_DEFAULT_NPATHS) const;

        void report_skew_setup(std::string filename, const tatum::SetupTimingAnalyzer& setup_analyzer, size_t nworst=REPORT_TIMING_DEFAULT_NPATHS) const;
        void report_skew_setup(std::ostream& os, const tatum::SetupTimingAnalyzer& setup_analyzer, size_t nworst=REPORT_TIMING_DEFAULT_NPATHS) const;

        void report_skew_hold(std::string filename, const tatum::HoldTimingAnalyzer& hold_analyzer, size_t nworst=REPORT_TIMING_DEFAULT_NPATHS) const;
        void report_skew_hold(std::ostream& os, const tatum::HoldTimingAnalyzer& hold_analyzer, size_t nworst=REPORT_TIMING_DEFAULT_NPATHS) const;

        void report_unconstrained_setup(std::string filename, const tatum::SetupTimingAnalyzer& setup_analyzer) const;
        void report_unconstrained_setup(std::ostream& os, const tatum::SetupTimingAnalyzer& setup_analyzer) const;

        void report_unconstrained_hold(std::string filename, const tatum::HoldTimingAnalyzer& hold_analyzer) const;
        void report_unconstrained_hold(std::ostream& os, const tatum::HoldTimingAnalyzer& hold_analyzer) const;

    private:
        struct PathSkew {
            NodeId launch_node;
            NodeId capture_node;
            DomainId launch_domain;
            DomainId capture_domain;
            Time clock_launch;
            Time clock_capture;
            Time clock_capture_normalized;
            Time clock_constraint;
            Time clock_skew;
        };

    private:
        void report_timing(std::ostream& os, const std::vector<TimingPath>& paths) const;

        void report_path(std::ostream& os, const TimingPath& path) const;

        void report_unconstrained(std::ostream& os, const NodeType type, const detail::TagRetriever& tag_retriever) const;

        void report_skew(std::ostream& os, const detail::TagRetriever& tag_retriever, TimingType timing_type, size_t nworst) const;

        void report_skew_path(std::ostream& os, const PathSkew& path_skew, TimingType timing_type) const;

        bool nearly_equal(const tatum::Time& lhs, const tatum::Time& rhs) const;

        size_t estimate_point_print_width(const TimingPath& path) const;

    private:
        const TimingGraphNameResolver& name_resolver_;
        const TimingGraph& timing_graph_;
        const TimingConstraints& timing_constraints_;
        float unit_scale_ = 1e-9;
        size_t precision_ = 3;

        float relative_error_tolerance_ = 1.e-5;
        float absolute_error_tolerance_ = 1e-13; //Sub pico-second

        TimingPathCollector path_collector_;
};

} //namespace
#endif
