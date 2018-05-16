#ifndef TATUM_TIMING_PATH_TRACING_HPP
#define TATUM_TIMING_PATH_TRACING_HPP
#include "tatum/TimingGraphFwd.hpp"
#include "tatum/report/TimingReportTagRetriever.hpp"
#include "TimingPath.hpp"

namespace tatum { namespace detail {

TimingPath trace_path(const TimingGraph& timing_graph,
                      const detail::TagRetriever& tag_retriever, 
                      const DomainId launch_domain,
                      const DomainId capture_domain,
                      const NodeId sink_node);

TimingSubPath trace_clock_launch_path(const TimingGraph& timing_graph,
                                      const detail::TagRetriever& tag_retriever, 
                                      const DomainId launch_domain,
                                      const DomainId capture_domain,
                                      const NodeId sink_node);

TimingSubPath trace_data_arrival_path(const TimingGraph& timing_graph,
                                      const detail::TagRetriever& tag_retriever, 
                                      const DomainId launch_domain,
                                      const DomainId capture_domain,
                                      const NodeId sink_node);

TimingSubPath trace_clock_capture_path(const TimingGraph& timing_graph,
                                       const detail::TagRetriever& tag_retriever, 
                                       const DomainId launch_domain,
                                       const DomainId capture_domain,
                                       const NodeId sink_node);
}} //namespace
#endif
