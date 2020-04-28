#pragma once
#include <algorithm>
#include "tatum/graph_walkers/TimingGraphWalker.hpp"
#include "tatum/TimingGraph.hpp"
#include "tatum/delay_calc/DelayCalculator.hpp"
#include "tatum/graph_visitors/GraphVisitor.hpp"

namespace tatum {

/**
 * A simple serial graph walker which traverses the timing graph in a levelized
 * manner.
 */
class SerialIncrWalker : public TimingGraphWalker {
    protected:
        void invalidate_edge_impl(const EdgeId edge) override {
            invalidated_edges_.push_back(edge);
        }

        void clear_invalidated_edges_impl() override {
            invalidated_edges_.clear();
        }

        void do_arrival_pre_traversal_impl(const TimingGraph& tg, const TimingConstraints& tc, GraphVisitor& visitor) override {
            size_t num_unconstrained = 0;

            LevelId first_level = *tg.levels().begin();
            for(NodeId node_id : tg.level_nodes(first_level)) {
                bool constrained = visitor.do_arrival_pre_traverse_node(tg, tc, node_id);

                if(!constrained) {
                    ++num_unconstrained;
                }
            }

            num_unconstrained_startpoints_ = num_unconstrained;
        }

        void do_required_pre_traversal_impl(const TimingGraph& tg, const TimingConstraints& tc, GraphVisitor& visitor) override {
            size_t num_unconstrained = 0;

            for(NodeId node_id : tg.logical_outputs()) {
                bool constrained = visitor.do_required_pre_traverse_node(tg, tc, node_id);

                if(!constrained) {
                    ++num_unconstrained;
                }
            }

            num_unconstrained_endpoints_ = num_unconstrained;
        }

        void do_arrival_traversal_impl(const TimingGraph& tg, const TimingConstraints& tc, const DelayCalculator& dc, GraphVisitor& visitor) override {
            resize_incr_update_levels(tg);
            prepare_incr_arrival_update(tg, visitor);

            std::cout << "Arr Levels " << incr_arr_update_.min_level << ": " << incr_arr_update_.max_level << "\n";
            for(int level_idx = incr_arr_update_.min_level; level_idx <= incr_arr_update_.max_level; ++level_idx) {
                LevelId level(level_idx);

                auto& level_nodes = incr_arr_update_.nodes_to_process[level_idx];

                uniquify(level_nodes);

                std::cout << "Processing Arr Level " << size_t(level) << ": " << level_nodes.size() << " nodes\n";

                for (NodeId node : level_nodes) {

                    //std::cout << "  Processing Arr " << node << "\n";

                    bool node_updated = visitor.do_arrival_traverse_node(tg, tc, dc, node);

                    if (node_updated) {
                        //Record that this node was updated, for later efficient slack update
                        enqueue_slack_node(node);

                        //Queue this node's downstream dependencies for updating
                        for (EdgeId edge : tg.node_out_edges(node)) {
                            NodeId snk_node = tg.edge_sink_node(edge);
                            enqueue_arr_node(tg, snk_node, edge, visitor);
                        }
                    }
                }
            }
            std::cout  << "Arr Processed " << incr_arr_update_.total_levels_to_process() << " levels, " << incr_arr_update_.total_nodes_to_process() << " nodes\n";
            complete_incr_arrival_update(tg);
        }

        void do_required_traversal_impl(const TimingGraph& tg, const TimingConstraints& tc, const DelayCalculator& dc, GraphVisitor& visitor) override {
            resize_incr_update_levels(tg);
            prepare_incr_required_update(tg, visitor);

            std::cout << "Req Levels " << incr_req_update_.max_level << ": " << incr_req_update_.min_level << "\n";
            for(int level_idx = incr_req_update_.max_level; level_idx >= incr_req_update_.min_level; --level_idx) {
                LevelId level(level_idx);

                auto& level_nodes = incr_req_update_.nodes_to_process[level_idx];

                uniquify(level_nodes);

                std::cout << "Processing Req Level " << size_t(level) << ": " << level_nodes.size() << " nodes\n";

                for (NodeId node : level_nodes) {

                    //std::cout << "  Processing Req " << node << "\n";

                    bool node_updated = visitor.do_required_traverse_node(tg, tc, dc, node);

                    if (node_updated) {

                        //std::cout << "    Enqueing slack\n";

                        //Record that this node was updated, for later efficient slack update
                        enqueue_slack_node(node);

                        //Queue this node's downstream dependencies for updating
                        for (EdgeId edge : tg.node_in_edges(node)) {
                            NodeId src_node = tg.edge_src_node(edge);

                            enqueue_req_node(tg, src_node, edge, visitor);
                        }
                    }
                }
            }
            std::cout  << "Req Processed " << incr_req_update_.total_levels_to_process() << " levels, " << incr_req_update_.total_nodes_to_process() << " nodes\n";

            complete_incr_required_update(tg);
        }

        void do_update_slack_impl(const TimingGraph& tg, const DelayCalculator& dc, GraphVisitor& visitor) override {
            uniquify(nodes_to_update_slack_);

            std::cout << "Processing slack updates for " << nodes_to_update_slack_.size() << " nodes\n";

            for(NodeId node : nodes_to_update_slack_) {
                //std::cout << "  Processing slack " << node << "\n";

                for (EdgeId edge : tg.node_in_edges(node)) {
                    visitor.do_reset_edge(edge);
                }
                visitor.do_reset_node_slack_tags(node);

                visitor.do_slack_traverse_node(tg, dc, node);
            }

            nodes_to_update_slack_.clear();
        }

        void do_reset_impl(const TimingGraph& tg, GraphVisitor& visitor) override {
            for(NodeId node_id : tg.nodes()) {
                visitor.do_reset_node(node_id);
            }
            for(EdgeId edge_id : tg.edges()) {
                visitor.do_reset_edge(edge_id);
            }
        }

        size_t num_unconstrained_startpoints_impl() const override { return num_unconstrained_startpoints_; }
        size_t num_unconstrained_endpoints_impl() const override { return num_unconstrained_endpoints_; }
    private:

        void uniquify(std::vector<NodeId>& nodes) {
            std::sort(nodes.begin(), nodes.end());
            nodes.erase(std::unique(nodes.begin(), nodes.end()),
                        nodes.end());
        }

        void prepare_incr_arrival_update(const TimingGraph& tg, GraphVisitor& visitor) {
            incr_arr_update_.min_level = size_t(*(tg.levels().end() - 1));
            incr_arr_update_.max_level = size_t(*tg.levels().begin());
            incr_req_update_.min_level = size_t(*(tg.levels().end() - 1));
            incr_req_update_.max_level = size_t(*tg.levels().begin());

            std::cout << "Invalidated Edges: " << invalidated_edges_.size() << " / " << tg.edges().size() << " (" << invalidated_edges_.size() / tg.edges().size() << ")\n";
            for (EdgeId edge : invalidated_edges_) {
                NodeId node = tg.edge_sink_node(edge);

                enqueue_arr_node(tg, node, edge, visitor);
            }
        }

        void complete_incr_arrival_update(const TimingGraph& tg) {
            for (int level = incr_arr_update_.min_level; level <= incr_arr_update_.max_level; ++level) {
                incr_arr_update_.nodes_to_process[level].clear();
            }

            incr_arr_update_.min_level = size_t(*(tg.levels().end() - 1));
            incr_arr_update_.max_level = size_t(*tg.levels().begin());
        }
        void prepare_incr_required_update(const TimingGraph& tg, GraphVisitor& visitor) {
            for (EdgeId edge : invalidated_edges_) {
                NodeId node = tg.edge_src_node(edge);

                enqueue_req_node(tg, node, edge, visitor);
            }
        }

        void complete_incr_required_update(const TimingGraph& tg) {
            for (int level = incr_req_update_.min_level; level <= incr_req_update_.max_level; ++level) {
                incr_req_update_.nodes_to_process[level].clear();
            }

            incr_req_update_.min_level = size_t(*(tg.levels().end() - 1));
            incr_req_update_.max_level = size_t(*tg.levels().begin());
        }

        void enqueue_arr_node(const TimingGraph& tg, NodeId node, EdgeId invalidated_edge, GraphVisitor& visitor) {
            if (0) { //Block invalidation
                visitor.do_reset_node_arrival_tags(node);
            } else { //Edge invalidation
                //Data arrival tags track their associated origin node (i.e. dominant
                //edge which determines the tag value). Rather than invalidate all
                //tags to ensure the correctly updated tag when the node is re-traversed,
                //we can get away with only invalidating the tag when the dominant edge
                //is invalidated.
                //
                //This ensures that cases where non-dominate edges change delay value,
                //but not in ways which effect the tag value, we detect that the tag
                //is 'unchanged', which helps keep the number of updated nodes small.
                NodeId src_node = tg.edge_src_node(invalidated_edge);
                visitor.do_reset_node_arrival_tags_from_origin(node, src_node);


                //At SOURCE/SINK nodes clock launch/capture tags are converted into
                //data arrival/required tags, so we also need to carefully reset those
                //tags as well (since they don't track origin nodes this must be done
                //seperately).
                EdgeType edge_type = tg.edge_type(invalidated_edge);
                if (edge_type == EdgeType::PRIMITIVE_CLOCK_CAPTURE) {
                    //We mark required times on sinks based the clock capture time during
                    //the arrival traversal.
                    //
                    //Therefore we need to invalidate the data required times (so they are
                    //re-calculated correctly) when the clock caputre edge has been 
                    //invalidated. Note that we don't need to enqueue the sink node for 
                    //required time traversal, since the required time will be updated during
                    //the arrival traversasl.
                    visitor.do_reset_node_required_tags(node);

                    //However, since the required time traversal doesn't update the sink node,
                    //it's dependent nodes won't be enqueued for processing, even if the required
                    //times changed, so we explicitly do that here
                    for (EdgeId sink_in_edge : tg.node_in_edges(node)) {
                        NodeId sink_src_node = tg.edge_src_node(sink_in_edge);
                        enqueue_req_node(tg, sink_src_node, sink_in_edge, visitor);
                    }
                } else if (edge_type == EdgeType::PRIMITIVE_CLOCK_LAUNCH) {
                    //On propagating to a SOURCE node, CLOCK_LAUNCH becomes DATA_ARRIVAL
                    //we therefore invalidate any outstanding arrival tags if the clock launch
                    //edge has been invalidated.
                    //
                    //This ensures the correct data arrivaltags are generated
                    visitor.do_reset_node_arrival_tags(node);
                }

            }

            //std::cout << "  Enqueing arr " << node << "\n";
            incr_arr_update_.enqueue_node(tg, node);
        }

        void enqueue_req_node(const TimingGraph& tg, NodeId node, EdgeId invalidated_edge, GraphVisitor& visitor) {
            if (0) { //Block invalidation
                visitor.do_reset_node_required_tags(node);
            } else { //Edge invalidation
                NodeId snk_node = tg.edge_sink_node(invalidated_edge);
                visitor.do_reset_node_required_tags_from_origin(node, snk_node);
            }

            //std::cout << "  Enqueing req " << node << "\n";
            incr_req_update_.enqueue_node(tg, node);
        }

        void enqueue_slack_node(const NodeId node) {
            nodes_to_update_slack_.push_back(node); 
        }

        void resize_incr_update_levels(const TimingGraph& tg) {
            incr_arr_update_.nodes_to_process.resize(tg.levels().size());
            incr_req_update_.nodes_to_process.resize(tg.levels().size());
        }

        struct t_incr_traversal_update {
            std::vector<std::vector<NodeId>> nodes_to_process;
            int min_level = 0;
            int max_level = 0;

            void enqueue_node(const TimingGraph& tg, NodeId node) {
                int level = size_t(tg.node_level(node));

                nodes_to_process[level].push_back(node);
                min_level = std::min(min_level, level);
                max_level = std::max(max_level, level);
            }

            size_t total_nodes_to_process() {
                size_t cnt = 0;
                for (int level = min_level; level <= max_level; ++level) {
                    cnt += nodes_to_process[level].size();
                }
                return cnt;
            }

            size_t total_levels_to_process() {
                return size_t(max_level) - size_t(min_level) + 1;
            }
        };

        t_incr_traversal_update incr_arr_update_;
        t_incr_traversal_update incr_req_update_;
        std::vector<NodeId> nodes_to_update_slack_;

        std::vector<EdgeId> invalidated_edges_;

        size_t num_unconstrained_startpoints_ = 0;
        size_t num_unconstrained_endpoints_ = 0;
};

} //namepsace
