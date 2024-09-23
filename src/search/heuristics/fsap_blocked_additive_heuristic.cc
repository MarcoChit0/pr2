#include "fsap_blocked_additive_heuristic.h"

#include "../global_operator.h"
#include "../option_parser.h"
#include "../plugin.h"
#include "../global_state.h"

#include <cassert>
#include <vector>

using namespace std;


namespace FSAPBlockedAdditiveHeuristic {

// construction and destruction
FSAPBlockedAdditiveHeuristic::FSAPBlockedAdditiveHeuristic(const Options &opts)
    : RelaxationHeuristic(opts),
      did_write_overflow_warning(false) {
    g_heuristic_for_reachability = this;
}

FSAPBlockedAdditiveHeuristic::~FSAPBlockedAdditiveHeuristic() {
}

void FSAPBlockedAdditiveHeuristic::write_overflow_warning() {
    if (!did_write_overflow_warning) {
        // TODO: Should have a planner-wide warning mechanism to handle
        // things like this.
        cout << "WARNING: overflow on h^add! Costs clamped to "
             << MAX_COST_VALUE << endl;
        cerr << "WARNING: overflow on h^add! Costs clamped to "
             << MAX_COST_VALUE << endl;
        did_write_overflow_warning = true;
    }
}

// initialization
void FSAPBlockedAdditiveHeuristic::initialize() {
    if (!g_silent_planning)
        cout << "Initializing additive heuristic..." << endl;
    RelaxationHeuristic::initialize();
}

// heuristic computation
void FSAPBlockedAdditiveHeuristic::setup_exploration_queue() {
    queue.clear();

    for (int var = 0; var < propositions.size(); var++) {
        for (int value = 0; value < propositions[var].size(); value++) {
            Proposition &prop = propositions[var][value];
            prop.cost = -1;
            prop.marked = false;
        }
    }

    // Deal with operators and axioms without preconditions.
    for (int i = 0; i < unary_operators.size(); i++) {
        UnaryOperator &op = unary_operators[i];

        op.unsatisfied_preconditions = op.precondition.size();
        op.cost = op.base_cost; // will be increased by precondition costs

        if (op.unsatisfied_preconditions == 0)
            enqueue_if_necessary(op.effect, op.base_cost, &op);
    }
}

void FSAPBlockedAdditiveHeuristic::setup_exploration_queue_state(const StateInterface &state) {
    for (int var = 0; var < propositions.size(); var++) {
        if (-1 == state[var]) {
            for (int val = 0; val < propositions[var].size(); val++) {
                Proposition *init_prop = &propositions[var][val];
                enqueue_if_necessary(init_prop, 0, 0);
            }
        } else {
            Proposition *init_prop = &propositions[var][state[var]];
            enqueue_if_necessary(init_prop, 0, 0);
        }
    }
}

bool FSAPBlockedAdditiveHeuristic::relaxed_exploration(bool include_forbidden) {
    int unsolved_goals = goal_propositions.size();
    while (!queue.empty()) {
        pair<int, Proposition *> top_pair = queue.pop();
        int distance = top_pair.first;
        Proposition *prop = top_pair.second;
        int prop_cost = prop->cost;
        assert(prop_cost >= 0);
        assert(prop_cost <= distance);
        if (prop_cost < distance)
            continue;
        if (prop->is_goal && --unsolved_goals == 0)
            return true;
        const vector<UnaryOperator *> &triggered_operators =
            prop->precondition_of;
        for (int i = 0; i < triggered_operators.size(); i++) {

            UnaryOperator *unary_op = triggered_operators[i];
            increase_cost(unary_op->cost, prop_cost);
            unary_op->unsatisfied_preconditions--;

            // HAZ: This assertion no longer holds with forbidden operators
            assert(unary_op->unsatisfied_preconditions >= 0);





            // TODO: If the unary_op has all of its preconditions satisfied
            //       then it should not be forbidden (make this an assumption
            //       below). When enqueing, if it's a precondition of the
            //       action, then all matching operators should be erased.
            //       If it's just a condition in the fsap, then only ones
            //       matching the fsaps should be erased. This probably
            //       needs to be rethought.

            // TODO: Currently there is an issue with the approach using
            //       enqueue_if_necessary. When a forbidden operator is
            //       detected as applicable and ready, is_forbidden may
            //       return true, and then the enqueue never occur. Then
            //       later in the process a new prop may remove the same
            //       operator from the forbidden list through an enqueue
            //       while at the same time we'll never trigger the same
            //       operator because it was re-enabled through an fsap
            //       condition that isn't also a precondition. Thus, we
            //       may technically have a situation where an operator
            //       is permanently forbidden, even if it is usable in
            //       the future.




            if (unary_op->unsatisfied_preconditions == 0) {

                bool is_forbidden = (0 != forbidden_ops.count(g_operators[unary_op->operator_no].nondet_index));

                if (!g_detect_deadends || !is_forbidden)
                    enqueue_if_necessary(unary_op->effect,
                                         unary_op->cost,
                                         unary_op, include_forbidden);
                else assert(unary_op->cost == unary_op->base_cost);

            }
        }
    }
    return false;
}

void FSAPBlockedAdditiveHeuristic::mark_preferred_operators(
    const GlobalState &state, Proposition *goal) {
    if (!goal->marked) { // Only consider each subgoal once.
        goal->marked = true;
        UnaryOperator *unary_op = goal->reached_by;
        if (unary_op) { // We have not yet chained back to a start node.
            for (int i = 0; i < unary_op->precondition.size(); i++)
                mark_preferred_operators(state, unary_op->precondition[i]);
            int operator_no = unary_op->operator_no;
            if (unary_op->cost == unary_op->base_cost && operator_no != -1) {
                // Necessary condition for this being a preferred
                // operator, which we use as a quick test before the
                // more expensive applicability test.
                // If we had no 0-cost operators and axioms to worry
                // about, this would also be a sufficient condition.
                const Operator *op = &g_operators[operator_no];
                if (op->is_applicable(state))
                    set_preferred(op);
            }
        }
    }
}

int FSAPBlockedAdditiveHeuristic::compute_add_and_ff(const StateInterface &state) {

    setup_exploration_queue();
    setup_exploration_queue_state(state);
    bool worked = relaxed_exploration(false);

    if (g_check_with_forbidden && !worked) {
        setup_exploration_queue();
        setup_exploration_queue_state(state);
        relaxed_exploration(true);
    }

    int total_cost = 0;
    for (int i = 0; i < goal_propositions.size(); i++) {
        int prop_cost = goal_propositions[i]->cost;
        if (prop_cost == -1)
            return DEAD_END;
        increase_cost(total_cost, prop_cost);
    }
    return total_cost;
}

int FSAPBlockedAdditiveHeuristic::compute_heuristic(const GlobalState &state) {

    compute_forbidden(state);

    int h = compute_add_and_ff(state);
    if (h != DEAD_END) {
        for (int i = 0; i < goal_propositions.size(); i++)
            mark_preferred_operators(state, goal_propositions[i]);
    }
    return h;
}

static Heuristic *_parse(OptionParser &parser) {
    parser.document_synopsis("FSAP Blocked Additive heuristic", "");
    parser.document_language_support("action costs", "supported");
    parser.document_language_support("conditional_effects", "supported");
    parser.document_language_support(
        "axioms",
        "supported (in the sense that the planner won't complain -- "
        "handling of axioms might be very stupid "
        "and even render the heuristic unsafe)");
    parser.document_property("admissible", "no");
    parser.document_property("consistent", "no");
    parser.document_property("safe", "yes for tasks without axioms");
    parser.document_property("preferred operators", "yes");

    Heuristic::add_options_to_parser(parser);
    Options opts = parser.parse();
    if (parser.dry_run())
        return 0;
    else
        return new FSAPBlockedAdditiveHeuristic(opts);
}

static Plugin<Heuristic> _plugin("fsapblockadd", _parse);

}