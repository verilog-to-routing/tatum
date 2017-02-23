#include <map>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <vector>

#include "tatum/util/tatum_math.hpp"
#include "tatum/util/tatum_error.hpp"
#include "tatum/TimingReporter.hpp"
#include "tatum/TimingGraph.hpp"
#include "tatum/TimingConstraints.hpp"

namespace tatum {

TimingReporter::TimingReporter(const TimingGraphNameResolver& name_resolver,
                               const TimingGraph& timing_graph, 
                               const TimingConstraints& timing_constraints, 
                               float unit_scale,
                               size_t precision)
    : name_resolver_(name_resolver)
    , timing_graph_(timing_graph)
    , timing_constraints_(timing_constraints)
    , unit_scale_(unit_scale)
    , precision_(precision) {
    //pass
}

void TimingReporter::report_timing_setup(std::string filename, 
                                         const SetupTimingAnalyzer& setup_analyzer,
                                         size_t npaths) const {
    std::ofstream os(filename);
    report_timing_setup(os, setup_analyzer, npaths);
}

void TimingReporter::report_timing_setup(std::ostream& os, 
                                         const SetupTimingAnalyzer& setup_analyzer,
                                         size_t npaths) const {
    detail::SetupTagRetriever tag_retriever(setup_analyzer);
    auto paths = collect_worst_paths(tag_retriever, npaths);

    report_timing(os, paths, npaths);
}

void TimingReporter::report_timing_hold(std::string filename, 
                                         const HoldTimingAnalyzer& hold_analyzer,
                                         size_t npaths) const {
    std::ofstream os(filename);
    report_timing_hold(os, hold_analyzer, npaths);
}

void TimingReporter::report_timing_hold(std::ostream& os, 
                                         const HoldTimingAnalyzer& hold_analyzer,
                                         size_t npaths) const {
    detail::HoldTagRetriever tag_retriever(hold_analyzer);
    auto paths = collect_worst_paths(tag_retriever, npaths);

    report_timing(os, paths, npaths);
}

void TimingReporter::report_timing(std::ostream& os,
                                   const std::vector<TimingPath>& paths, 
                                   size_t npaths) const {
    os << "#Timing report of worst " << npaths << " path(s)\n";
    os << "# Unit scale: " << std::setprecision(0) << std::scientific << unit_scale_ << " seconds\n";
    os << "# Output precision: " << precision_ << "\n";
    os << "\n";

    size_t i = 0;
    for(const auto& path : paths) {
        os << "#Path " << ++i << "\n";
        report_path(os, path);
        os << "\n";
    }

    os << "#End of timing report\n";
}

void TimingReporter::report_path(std::ostream& os, const TimingPath& timing_path) const {
    std::string divider = "--------------------------------------------------------------------------------";

    os << "Startpoint: " << name_resolver_.node_name(timing_path.startpoint()) 
       << " (" << name_resolver_.node_block_type_name(timing_path.startpoint())
        << " clocked by " << timing_constraints_.clock_domain_name(timing_path.launch_domain)
        << ")\n";
    os << "Endpoint  : " << name_resolver_.node_name(timing_path.endpoint()) 
        << " (" << name_resolver_.node_block_type_name(timing_path.endpoint()) 
        << " clocked by " << timing_constraints_.clock_domain_name(timing_path.capture_domain)
        << ")\n";

    if(timing_path.type == TimingPathType::SETUP) {
        os << "Path Type : max [setup]" << "\n";
    } else {
        TATUM_ASSERT_MSG(timing_path.type == TimingPathType::HOLD, "Expected path type SETUP or HOLD");
        os << "Path Type : min [hold]" << "\n";
    }

    os << "\n";
    print_path_line(os, "Point", " Incr", " Path");
    os << divider << "\n";

    //Launch path
    Time arr_time;
    Time arr_path;
    {
        reset_path();

        {
            //Launch clock origin
            arr_path = Time(0.);
            std::string point = "clock " + timing_constraints_.clock_domain_name(timing_path.launch_domain) + " (rise edge)";
            update_print_path(os, point, arr_path);
        }

        {
            //Launch clock latency
            Time latency = Time(timing_constraints_.source_latency(timing_path.launch_domain));
            arr_path += latency;
            std::string point = "clock source latency";
            update_print_path(os, point, arr_path);
        }

        //Launch clock path
        for(TimingPathElem path_elem : timing_path.clock_launch_elements) {
            std::string point = name_resolver_.node_name(path_elem.node) + " (" + name_resolver_.node_block_type_name(path_elem.node) + ")";
            arr_path = path_elem.tag.time();

            update_print_path(os, point, arr_path);
        }

        {
            //Input constraint
            TATUM_ASSERT(timing_path.data_arrival_elements.size() > 0);
            const TimingPathElem& path_elem = timing_path.data_arrival_elements[0];

            float input_constraint = timing_constraints_.input_constraint(path_elem.node, timing_path.launch_domain);
            if(!std::isnan(input_constraint)) {
                arr_path += Time(input_constraint);

                update_print_path(os, "input external delay", arr_path);
            }
        }

        //Launch data
        for(size_t ielem = 0; ielem < timing_path.data_arrival_elements.size(); ++ielem) {
            const TimingPathElem& path_elem = timing_path.data_arrival_elements[ielem];

            std::string point = name_resolver_.node_name(path_elem.node) + " (" + name_resolver_.node_block_type_name(path_elem.node) + ")";

            if(path_elem.incomming_edge 
               && timing_graph_.edge_type(path_elem.incomming_edge) == EdgeType::PRIMITIVE_CLOCK_LAUNCH) {
                    point += " [clk-to-q]";
            }

            arr_path = path_elem.tag.time();

            update_print_path(os, point, arr_path);
        }

        {
            //Final arrival time
            const auto& last_tag = timing_path.data_arrival_elements[timing_path.data_arrival_elements.size() - 1].tag;
            TATUM_ASSERT(last_tag.type() == TagType::DATA_ARRIVAL);
            arr_time = last_tag.time();
            update_print_path_no_incr(os, "data arrival time", arr_time);
            os << "\n";
        }

        //Sanity check the arrival time calculated by this timing report (i.e. path) and the one calculated by
        //the analyzer (i.e. arr_time) agree
        if(!nearly_equal(arr_time, arr_path)) {
            os.flush();
            std::stringstream ss;
            ss << "Internal Error: analyzer arrival time (" << arr_time.value() << ")"
               << " differs from timing report path arrival time (" << arr_path.value() << ")"
               << " beyond tolerance";
            throw tatum::Error(ss.str());
        }
    }


    //Capture path (required time)
    Time req_time;
    Time req_path;
    {
        reset_path();

        {
            //Capture clock origin
            req_path = Time(0.);

            Time constraint;
            if(timing_path.type == TimingPathType::SETUP) {
                constraint = Time(timing_constraints_.setup_constraint(timing_path.launch_domain, timing_path.capture_domain));
            } else {
                constraint = Time(timing_constraints_.hold_constraint(timing_path.launch_domain, timing_path.capture_domain));
            }

            req_path += constraint;
            std::string point = "clock " + timing_constraints_.clock_domain_name(timing_path.capture_domain) + " (rise edge)";
            update_print_path(os, point, req_path);
        }

        {
            //Capture clock source latency
            Time latency = Time(timing_constraints_.source_latency(timing_path.capture_domain));
            req_path += latency;
            std::string point = "clock source latency";
            update_print_path(os, point, req_path);
        }

        //Clock capture path
        for(size_t ielem = 0; ielem < timing_path.clock_capture_elements.size(); ++ielem) {
            const TimingPathElem& path_elem = timing_path.clock_capture_elements[ielem];

            TATUM_ASSERT(path_elem.tag.type() == TagType::CLOCK_CAPTURE);

            req_path = path_elem.tag.time();
            std::string point = name_resolver_.node_name(path_elem.node) + " (" + name_resolver_.node_block_type_name(path_elem.node) + ")";
            update_print_path(os, point, req_path);
        }

        //Data required
        {
            const TimingPathElem& path_elem = timing_path.data_required_element;
            if(path_elem.incomming_edge && timing_graph_.edge_type(path_elem.incomming_edge) == EdgeType::PRIMITIVE_CLOCK_CAPTURE) {
                std::string point;
                if(timing_path.type == TimingPathType::SETUP) {
                    point = "cell setup time";
                } else {
                    TATUM_ASSERT_MSG(timing_path.type == TimingPathType::HOLD, "Expected path type SETUP or HOLD");
                    point = "cell hold time";
                }
                req_path = path_elem.tag.time();
                update_print_path(os, point, req_path);
            }

            //Output constraint
            Time output_constraint = -Time(timing_constraints_.output_constraint(path_elem.node, timing_path.capture_domain));
            if(!std::isnan(output_constraint.value())) {
                req_path += output_constraint;
                update_print_path(os, "output external delay", req_path);
            }

            //Uncertainty
            Time uncertainty;
            if(timing_path.type == TimingPathType::SETUP) {
                uncertainty = -Time(timing_constraints_.setup_clock_uncertainty(timing_path.launch_domain, timing_path.capture_domain));
            } else {
                uncertainty = Time(timing_constraints_.hold_clock_uncertainty(timing_path.launch_domain, timing_path.capture_domain));
            }
            req_path += uncertainty;
            update_print_path(os, "clock uncertainty", req_path);

            //Final arrival time
            req_time = path_elem.tag.time();
            update_print_path_no_incr(os, "data required time", req_time);
        }


        //Sanity check required time
        if(!nearly_equal(req_time, req_path)) {
            os.flush();
            std::stringstream ss;
            ss << "Internal Error: analyzer required time (" << req_time.value() << ")"
               << " differs from report_timing path required time (" << req_path.value() << ")"
               << " beyond tolerance";
            throw tatum::Error(ss.str());
        }
    }

    //Summary and slack
    os << divider << "\n";
    print_path_line_no_incr(os, "data required time", req_time);
    print_path_line_no_incr(os, "data arrival time", -arr_time);
    os << divider << "\n";
    Time slack = timing_path.slack_tag.time();
    if(slack.value() < 0. || std::signbit(slack.value())) {
        print_path_line_no_incr(os, "slack (VIOLATED)", slack);
    } else {
        print_path_line_no_incr(os, "slack (MET)", slack);
    }
    os << "\n";
    os.flush();

    //Sanity check slack
    Time path_slack = req_path - arr_path;
    if(!nearly_equal(slack, path_slack)) {
        os.flush();
        std::stringstream ss;
        ss << "Internal Error: analyzer slack (" << slack << ")"
           << " differs from report_timing path slack (" << path_slack << ")"
           << " beyond tolerance";
        throw tatum::Error(ss.str());
    }
}

void TimingReporter::update_print_path(std::ostream& os, std::string point, Time path) const {

    Time incr = path - prev_path_;

    print_path_line(os, point, to_printable_string(incr), to_printable_string(path));

    prev_path_ = path;
}

void TimingReporter::update_print_path_no_incr(std::ostream& os, std::string point, Time path) const {
    TATUM_ASSERT(nearly_equal(path, prev_path_));

    print_path_line(os, point, "", to_printable_string(path));

    prev_path_ = path;
}

void TimingReporter::reset_path() const {
    prev_path_ = Time(0.);
}

void TimingReporter::print_path_line_no_incr(std::ostream& os, std::string point, Time path) const {
    print_path_line(os, point, "", to_printable_string(path));
}

void TimingReporter::print_path_line(std::ostream& os, std::string point, std::string incr, std::string path) const {
    os << std::setw(60) << std::left << point;
    os << std::setw(10) << std::left << incr;
    os << std::setw(10) << std::left << path;
    os << "\n";
}

std::vector<TimingPath> TimingReporter::collect_worst_paths(const detail::TagRetriever& tag_retriever, size_t npaths) const {
    std::vector<TimingPath> paths;

    //Sort the sinks by slack
    std::map<TimingTag,NodeId,TimingTagValueComp> tags_and_sinks;

    //Add the slacks of all sink
    for(NodeId node : timing_graph_.logical_outputs()) {
        for(TimingTag tag : tag_retriever.slacks(node)) {
            tags_and_sinks.emplace(tag,node);
        }
    }

    //Trace the paths for each tag/node pair
    // The the map will sort the nodes and tags from worst to best slack (i.e. ascending slack order),
    // so the first pair is the most critical end-point
    for(const auto& kv : tags_and_sinks) {
        NodeId sink_node;
        TimingTag sink_tag;
        std::tie(sink_tag, sink_node) = kv;

        TimingPath path = trace_path(tag_retriever, sink_tag, sink_node); 

        paths.push_back(path);

        if(paths.size() >= npaths) break;
    }

    return paths;
}

TimingPath TimingReporter::trace_path(const detail::TagRetriever& tag_retriever, 
                                      const TimingTag& sink_tag, 
                                      const NodeId sink_node) const {
    TATUM_ASSERT(timing_graph_.node_type(sink_node) == NodeType::SINK);
    TATUM_ASSERT(sink_tag.type() == TagType::SLACK);

    TimingPath path;
    path.launch_domain = sink_tag.launch_clock_domain();
    path.capture_domain = sink_tag.capture_clock_domain();
    path.type = tag_retriever.type();

    /*
     * Back-trace the data launch path
     */
    NodeId curr_node = sink_node;
    while(curr_node && timing_graph_.node_type(curr_node) != NodeType::CPIN) {
        //Trace until we hit the origin, or a clock pin

        if(curr_node == sink_node) {
            //Record the slack at the sink node
            auto slack_tags = tag_retriever.slacks(curr_node);
            auto iter = find_tag(slack_tags, path.launch_domain, path.capture_domain);
            TATUM_ASSERT(iter != slack_tags.end());

            path.slack_tag = *iter;
        }

        auto data_tags = tag_retriever.tags(curr_node, TagType::DATA_ARRIVAL);

        //First try to find the exact tag match
        auto iter = find_tag(data_tags, path.launch_domain, path.capture_domain);
        if(iter == data_tags.end()) {
            //Then look for incompletely specified (i.e. wildcard) capture clocks
            iter = find_tag(data_tags, path.launch_domain, DomainId::INVALID());
        }
        TATUM_ASSERT(iter != data_tags.end());

        EdgeId edge;
        if(iter->origin_node()) {
            edge = timing_graph_.find_edge(iter->origin_node(), curr_node);
            TATUM_ASSERT(edge);
        }

        //Record
        path.data_arrival_elements.emplace_back(*iter, curr_node, edge);

        //Advance to the previous node
        curr_node = iter->origin_node();
    }
    //Since we back-traced from sink we reverse to get the forward order
    std::reverse(path.data_arrival_elements.begin(), path.data_arrival_elements.end());

    /*
     * Back-trace the launch clock path
     */
    EdgeId clock_launch_edge = timing_graph_.node_clock_launch_edge(path.data_arrival_elements[0].node);
    if(clock_launch_edge) {
        //Mark the edge between clock and data paths
        path.data_arrival_elements[0].incomming_edge = clock_launch_edge;

        //Through the clock network
        curr_node = timing_graph_.edge_src_node(clock_launch_edge);
        TATUM_ASSERT(timing_graph_.node_type(curr_node) == NodeType::CPIN);

        while(curr_node) {
            auto launch_tags = tag_retriever.tags(curr_node, TagType::CLOCK_LAUNCH);
            auto iter = find_tag(launch_tags, path.launch_domain, path.capture_domain);
            if(iter == launch_tags.end()) {
                //Then look for incompletely specified (i.e. wildcard) capture clocks
                iter = find_tag(launch_tags, path.launch_domain, DomainId::INVALID());
            }
            TATUM_ASSERT(iter != launch_tags.end());

            EdgeId edge;
            if(iter->origin_node()) {
                edge = timing_graph_.find_edge(iter->origin_node(), curr_node);
                TATUM_ASSERT(edge);
            }

            //Record
            path.clock_launch_elements.emplace_back(*iter, curr_node, edge);

            //Advance to the previous node
            curr_node = iter->origin_node();
        }
    }
    //Reverse back-trace
    std::reverse(path.clock_launch_elements.begin(), path.clock_launch_elements.end());

    /*
     * Back-trace the clock capture path
     */

    //Record the required time
    auto required_tags = tag_retriever.tags(sink_node, TagType::DATA_REQUIRED);
    auto req_iter = find_tag(required_tags, path.launch_domain, path.capture_domain);
    TATUM_ASSERT(req_iter != required_tags.end());
    path.data_required_element = TimingPathElem(*req_iter, curr_node, EdgeId::INVALID());

    EdgeId clock_capture_edge = timing_graph_.node_clock_capture_edge(sink_node);

    if(clock_capture_edge) {
        //Mark the edge between clock and data paths (i.e. setup/hold edge)
        path.data_required_element.incomming_edge = clock_capture_edge;

        curr_node = timing_graph_.edge_src_node(clock_capture_edge);
        TATUM_ASSERT(timing_graph_.node_type(curr_node) == NodeType::CPIN);

        while(curr_node) {

            //Record the clock capture tag
            auto capture_tags = tag_retriever.tags(curr_node, TagType::CLOCK_CAPTURE);
            auto iter = find_tag(capture_tags, path.launch_domain, path.capture_domain);
            TATUM_ASSERT(iter != capture_tags.end());

            EdgeId edge;
            if(iter->origin_node()) {
                edge = timing_graph_.find_edge(iter->origin_node(), curr_node);
                TATUM_ASSERT(edge);
            }

            path.clock_capture_elements.emplace_back(*iter, curr_node, edge);

            //Advance to the previous node
            curr_node = iter->origin_node();
        }
    }
    //Reverse back-trace
    std::reverse(path.clock_capture_elements.begin(), path.clock_capture_elements.end());

    return path;
}

float TimingReporter::convert_to_printable_units(float value) const {
    return value / unit_scale_;
}

std::string TimingReporter::to_printable_string(Time val) const {
    std::stringstream ss;
    if(!std::signbit(val.value())) ss << " "; //Pad possitive values so they align with negative prefixed with -
    ss << std::fixed << std::setprecision(precision_) << convert_to_printable_units(val.value());

    return ss.str();
}

bool TimingReporter::nearly_equal(const Time& lhs, const Time& rhs) const {
    return tatum::util::nearly_equal(lhs.value(), rhs.value(), absolute_error_tolerance_, relative_error_tolerance_);
}

} //namespace
