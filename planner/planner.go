package planner

import (
	"github.com/cuixudong74/goal-action-planner-go/world"
)

// Planner bundles the action library with search options. It is stateless
// and safe for concurrent use; every Plan call is independent.
type Planner struct {
	actions []*Action
	opts    Options
}

// New creates a Planner. The action slice is copied and sorted by name.
func New(actions []*Action, opts Options) *Planner {
	sorted := make([]*Action, len(actions))
	copy(sorted, actions)
	sortActions(sorted)
	return &Planner{actions: sorted, opts: opts.withDefaults()}
}

// Actions returns the sorted action library.
func (p *Planner) Actions() []*Action { return p.actions }

// Prepared is the outcome of goal selection and merging, ready to search.
type Prepared struct {
	// Goals are the eligible goals in deterministic selection order.
	Goals []Goal
	// Conditions is the merged goal condition set.
	Conditions []world.Condition
}

// Prepare resolves dependencies and checks conflicts. If the goals cannot
// be searched at all it returns a non-nil early Result describing why
// (dependency cycle, conflict, or trivially-empty goal set).
func (p *Planner) Prepare(goals []Goal) (Prepared, *Result) {
	eligible, cycle := resolveDependencies(goals)
	if cycle != nil {
		return Prepared{}, &Result{Status: StatusDependencyCycle, Cycle: cycle}
	}
	prep := Prepared{Goals: eligible, Conditions: mergeConditions(eligible)}
	if conflicts := detectConflicts(eligible); len(conflicts) > 0 {
		return prep, &Result{
			Status:    StatusGoalConflict,
			Goals:     goalNames(eligible),
			Conflicts: conflicts,
		}
	}
	return prep, nil
}

// NewSearch builds a resumable search for prepared goals from state.
func (p *Planner) NewSearch(state world.State, prep Prepared) *Search {
	return NewSearch(p.actions, state, prep.Conditions, p.opts)
}

// Finish converts a finished search Result into a full planning Result:
// it stamps the plan with its base version and goal names, and on
// unreachability classifies which individual goals are unreachable so the
// caller gets a distinguishable, actionable answer.
func (p *Planner) Finish(state world.State, version uint64, prep Prepared, res Result) Result {
	res.Goals = goalNames(prep.Goals)
	if res.Status == StatusOK {
		res.Plan.BaseVersion = version
		res.Plan.Goals = res.Goals
		return res
	}
	if res.Status == StatusUnreachable {
		res.UnreachableGoals = p.classifyUnreachable(state, prep)
	}
	return res
}

// classifyUnreachable searches each goal independently (in deterministic
// order) and lists those that cannot be reached on their own.
func (p *Planner) classifyUnreachable(state world.State, prep Prepared) []string {
	var out []string
	for _, g := range prep.Goals {
		s := NewSearch(p.actions, state, g.Conditions, p.opts)
		for !s.Step(1024) {
		}
		if s.Result().Status != StatusOK {
			out = append(out, g.Name)
		}
	}
	return out
}

// Plan is the one-shot blocking API: prepare, search to completion, finish.
func (p *Planner) Plan(state world.State, version uint64, goals []Goal) Result {
	prep, early := p.Prepare(goals)
	if early != nil {
		return *early
	}
	s := p.NewSearch(state, prep)
	for !s.Step(1024) {
	}
	return p.Finish(state, version, prep, s.Result())
}
