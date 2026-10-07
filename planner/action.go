// Package planner implements a deterministic goal-oriented action planner.
//
// Determinism rules (see README): among equivalent plans the planner picks
// the one with (1) lowest total cost, (2) fewest actions, (3) the
// lexicographically smallest action-name sequence. Search is uniform-cost
// with that exact ordering as the queue key, so results never depend on
// map iteration or scheduling order.
package planner

import (
	"github.com/cuixudong74/goal-action-planner-go/world"
)

// Action describes a single executable step.
type Action struct {
	Name string
	Cost float64

	// Preconditions must all hold for the action to be applicable.
	Preconditions []world.Condition
	// Effects are applied atomically when the action executes.
	Effects []world.Effect

	// Postconditions are verified by the runtime right after execution;
	// a violation means the effect did not materialize as expected and
	// triggers replanning. The planner itself ignores them.
	Postconditions []world.Condition

	// MutexKey groups actions that may not co-occur in a single plan.
	// Empty means no plan-level mutual exclusion.
	MutexKey string

	// Resource names an exclusive runtime resource the action needs while
	// executing. Empty means no resource is required.
	Resource string

	// Duration is how many scheduler ticks the action takes. Values <= 1
	// mean instant. Effects apply atomically at completion; interrupting
	// a long action aborts it with no partial effects.
	Duration int
}

// Goal is a desired set of conditions plus selection metadata.
type Goal struct {
	Name     string
	Priority int // higher wins during selection
	// Conditions define the desired world facts.
	Conditions []world.Condition
	// DependsOn lists goal names that must also be selected for this goal
	// to be eligible. Cycles are reported, not followed forever.
	DependsOn []string
}

// Plan is an ordered action sequence plus the context it was built from.
type Plan struct {
	Actions     []*Action
	TotalCost   float64
	BaseVersion uint64   // world version the plan was computed against
	Goals       []string // names of goals this plan satisfies
}

// Names returns the action names in order.
func (p Plan) Names() []string {
	n := make([]string, len(p.Actions))
	for i, a := range p.Actions {
		n[i] = a.Name
	}
	return n
}

// Status classifies a planning outcome. Failures with different causes are
// always distinguishable.
type Status int

const (
	StatusOK Status = iota
	// StatusGoalConflict: selected goals contradict each other.
	StatusGoalConflict
	// StatusUnreachable: goals are consistent but no action sequence reaches them.
	StatusUnreachable
	// StatusDependencyCycle: goal DependsOn relations form a cycle.
	StatusDependencyCycle
	// StatusBudgetExhausted: search hit its node/depth budget before finishing.
	StatusBudgetExhausted
)

func (s Status) String() string {
	switch s {
	case StatusOK:
		return "ok"
	case StatusGoalConflict:
		return "goal-conflict"
	case StatusUnreachable:
		return "unreachable"
	case StatusDependencyCycle:
		return "dependency-cycle"
	case StatusBudgetExhausted:
		return "budget-exhausted"
	}
	return "unknown"
}

// Conflict records a contradiction between two goals on one fact key.
type Conflict struct {
	Key   string
	GoalA string
	GoalB string
	CondA world.Condition
	CondB world.Condition
}

// Result is the full, inspectable outcome of a planning request.
type Result struct {
	Status Status
	Plan   Plan
	// Goals lists the goals that were merged into the attempt.
	Goals []string
	// Conflicts is populated when Status == StatusGoalConflict.
	Conflicts []Conflict
	// UnreachableGoals is populated when Status == StatusUnreachable.
	UnreachableGoals []string
	// Cycle is populated when Status == StatusDependencyCycle.
	Cycle []string
	// Expanded counts expanded search nodes (useful for tuning).
	Expanded int
}
