#include "prp_search_engine.h"

#include "../../option_parser.h"
#include "../../plugin.h"

#include "../../search_engines/search_common.h"
#include "../../search_engines/lazy_search.h"

#include "../../globals.h"
#include "../../open_list_factory.h"
#include "../../operator_cost.h"
#include "../../utils/memory.h"

#include "fsap_penalized_ff_heuristic.h"

#include <iostream>
#include <limits>

using namespace std;
using namespace utils;

namespace prp_search {

unique_ptr<SearchEngine> PRPSearch::get_search_engine() {
    /*
      Build the same search engine that would be generated by the
      command-line option "lazy_greedy(ff())". This is a bit complex
      because we need to manually set options that have default values
      when used from the command line and because "lazy_greedy" is a
      factory function with somewhat complex behaviour.
    */

    // Build FF heuristic object.
    if (h) {
        h->reset();
    } else {
        Options h_options;
        h_options.set<bool>("cache_estimates", true);
        h_options.set<shared_ptr<AbstractTask>>("transform", g_root_task());
        h = new fsap_penalized_ff_heuristic::FSAPPenalizedFFHeuristic(h_options);
        preferred_list.push_back(h);
        preferred_list_scalar.push_back(h);
    }

    // Build open list object.
    Options open_list_options;
    open_list_options.set("eval", h);
    open_list_options.set("evals", preferred_list_scalar);
    open_list_options.set("preferred", preferred_list);
    open_list_options.set("boost", 500);
    open_list_options.set("pref_only", false);
    shared_ptr<OpenListFactory> open_list = search_common::create_greedy_open_list_factory(open_list_options);

    // Build lazy search object.
    Options lazy_options;
    lazy_options.set<int>("cost_type", ONE);
    lazy_options.set<double>("max_time", numeric_limits<double>::infinity());
    lazy_options.set<int>("bound", numeric_limits<int>::max());
    lazy_options.set<shared_ptr<OpenListFactory>>("open", open_list);
    lazy_options.set<bool>("reopen_closed", false);
    lazy_options.set<bool>("randomize_successors", false);
    lazy_options.set<bool>("preferred_successors_first", true);
    lazy_options.set<int>("random_seed", -1);

    lazy_search::LazySearch * engine = new lazy_search::LazySearch(lazy_options);
    engine->set_pref_operator_heuristics(preferred_list);

    return unique_ptr<SearchEngine>(engine);
}

PRPSearch::PRPSearch(const Options &opts)
    : SearchEngine(opts),
      h(0) {}

SearchStatus PRPSearch::step() {

    unique_ptr<SearchEngine> current_search = get_search_engine();
    current_search->search();

    set_status(current_search->get_status());
    set_solution_found(current_search->found_solution());
    if (found_solution())
        set_plan(current_search->get_plan());

    if (PRP.logging.verbose) {
        if (current_search->found_solution()) {
            cout << "Plan found:" << endl;
            for (auto op : current_search->get_plan())
                cout << "- " << op->get_name() << "(" << op->get_cost() << ")" << endl;
        } else
            cout << "Planning failed." << endl;
    }
    return current_search->get_status();
}

void PRPSearch::print_statistics() const {
    cout << "Cumulative statistics:" << endl;
    statistics.print_detailed_statistics();
}

void PRPSearch::save_plan_if_necessary() const { }

static shared_ptr<SearchEngine> _parse(OptionParser &parser) {
    SearchEngine::add_options_to_parser(parser);
    Options opts = parser.parse();
    shared_ptr<PRPSearch> engine;

    if (!parser.dry_run())
        engine = make_shared<PRPSearch>(opts);

    return engine;
}

static PluginShared<SearchEngine> _plugin("prpsearch", _parse);
}
