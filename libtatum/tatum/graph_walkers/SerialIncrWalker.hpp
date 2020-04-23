#pragma once
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
            const auto& levels = tg.levels();
            auto level_itr = levels.begin() + 1;

            for(NodeId node_id : tg.level_nodes(*level_itr)) {
                next_level_nodes_to_process_->push_back(node_id);
            }

            while(prepare_to_traverse_next_level()) {
                TATUM_ASSERT(level_itr != levels.end());
                LevelId level_id = *level_itr++;

                std::cout << "Processing Arr Level " << size_t(level_id) << ": " << level_nodes_to_process_->size() << " nodes\n";

                for (NodeId node : *level_nodes_to_process_) {

                    bool node_updated = visitor.do_arrival_traverse_node(tg, tc, dc, node);

                    if (node_updated) {

                        //Record that this node was updated, for later efficient slack update
                        nodes_to_update_slack_.emplace_back(node);

                        //Queue this node's downstream dependencies for updating
                        for (EdgeId out_edge : tg.node_out_edges(node)) {
                            NodeId snk_node = tg.edge_sink_node(out_edge);
                            next_level_nodes_to_process_->push_back(snk_node);
                        }
                    }
                }
            }
        }

        void do_required_traversal_impl(const TimingGraph& tg, const TimingConstraints& tc, const DelayCalculator& dc, GraphVisitor& visitor) override {
            /*
             *for(LevelId level_id : tg.reversed_levels()) {
             *    for(NodeId node_id : tg.level_nodes(level_id)) {
             *        visitor.do_required_traverse_node(tg, tc, dc, node_id);
             *    }
             *}
             */

            const auto& rev_levels = tg.reversed_levels();
            auto rev_level_itr = rev_levels.begin() + 1;

            for(NodeId node_id : tg.level_nodes(*rev_level_itr)) {
                next_level_nodes_to_process_->push_back(node_id);
            }

            while(prepare_to_traverse_next_level()) {
                TATUM_ASSERT(rev_level_itr != rev_levels.end());
                LevelId level_id = *rev_level_itr++;

                std::cout << "Processing Req Level " << size_t(level_id) << ": " << level_nodes_to_process_->size() << " nodes\n";


                for (NodeId node : *level_nodes_to_process_) {

                    bool node_updated = visitor.do_required_traverse_node(tg, tc, dc, node);

                    if (node_updated) {

                        //Record that this node was updated, for later efficient slack update
                        nodes_to_update_slack_.emplace_back(node);

                        //Queue this node's downstream dependencies for updating
                        for (EdgeId in_edge : tg.node_in_edges(node)) {
                            NodeId src_node = tg.edge_src_node(in_edge);
                            next_level_nodes_to_process_->push_back(src_node);
                        }
                    }
                }
            }
        }

        void do_update_slack_impl(const TimingGraph& tg, const DelayCalculator& dc, GraphVisitor& visitor) override {
            uniquify(nodes_to_update_slack_);

            std::cout << "Processing slack updates for " << nodes_to_update_slack_.size() << " nodes\n";

            for(NodeId node : nodes_to_update_slack_) {
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

        bool prepare_to_traverse_next_level() {
            //Clear the nodes processed at the current level
            level_nodes_to_process_->clear();

            //Sort an uniquify nodes to update at next level
            uniquify(*next_level_nodes_to_process_);

            //Flip nodes to process for next level queues
            std::swap(level_nodes_to_process_, next_level_nodes_to_process_);

            return !level_nodes_to_process_->empty();
        }

        void uniquify(std::vector<NodeId>& nodes) {
            std::sort(nodes.begin(), nodes.end());
            nodes.erase(std::unique(nodes.begin(), nodes.end()),
                        nodes.end());
        }

        size_t num_unconstrained_startpoints_ = 0;
        size_t num_unconstrained_endpoints_ = 0;

        std::unique_ptr<std::vector<NodeId>> level_nodes_to_process_ = std::make_unique<std::vector<NodeId>>();
        std::unique_ptr<std::vector<NodeId>> next_level_nodes_to_process_ = std::make_unique<std::vector<NodeId>>();
        std::vector<NodeId> nodes_to_update_slack_;
};

} //namepsace
