#ifndef VPR_TIMING_REPORTER_H
#define VPR_TIMING_REPORTER_H
#include <iosfwd>
#include "timing_graph_fwd.hpp"
#include "timing_constraints_fwd.hpp"
#include "timing_analyzers.hpp"

#include "TimingGraphNameResolver.hpp"
#include "TimingPath.hpp"

namespace tatum {

//A class for generating timing reports
class TimingReporter {
    public:
        TimingReporter(const TimingGraphNameResolver& name_resolver,
                       const tatum::TimingGraph& timing_graph, 
                       const tatum::TimingConstraints& timing_constraints, 
                       float unit_scale=1e-9,
                       size_t precision=3);
    public:
        void report_timing_setup(std::string filename, const tatum::SetupTimingAnalyzer& setup_analyzer, size_t npaths=100) const;
        void report_timing_setup(std::ostream& os, const tatum::SetupTimingAnalyzer& setup_analyzer, size_t npaths=100) const;

        //TODO: implement report_timing_hold
    private:
        void report_timing(std::ostream& os, const std::vector<TimingPath>& paths, size_t npaths) const;
        void report_path(std::ostream& os, const TimingPath& path) const;
        void print_path_line(std::ostream& os, std::string point, tatum::Time incr, tatum::Time path) const;
        void print_path_line(std::ostream& os, std::string point, tatum::Time path) const;
        void print_path_line(std::ostream& os, std::string point, std::string incr, std::string path) const;

        std::vector<TimingPath> collect_worst_setup_paths(const tatum::SetupTimingAnalyzer& setup_analyzer, size_t npaths) const;

        TimingPath trace_setup_path(const tatum::SetupTimingAnalyzer& setup_analyzer, const tatum::TimingTag& sink_tag, const tatum::NodeId sink_node) const;

        float convert_to_printable_units(float) const;

        std::string to_printable_string(tatum::Time val) const;

    private:
        const TimingGraphNameResolver& name_resolver_;
        const tatum::TimingGraph& timing_graph_;
        const tatum::TimingConstraints& timing_constraints_;
        float unit_scale_;
        size_t precision_;
};

} //namespace
#endif
