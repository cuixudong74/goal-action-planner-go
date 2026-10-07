package goap

// Goal describes a desired set of facts with a priority and optional
// dependencies on other goals (by name). A goal is only eligible for
// planning once all goals it depends on are satisfied in the current
// state or planned for earlier in the same merged plan.
type Goal struct {
	Name      string
	Priority  int // higher wins when goals compete
	Desired   []Condition
	DependsOn []string
}

// GoalStatus is the distinguishable conclusion of evaluating a goal.
type GoalStatus int

const (
	// GoalSatisfied means the desired facts already hold; no plan needed.
	GoalSatisfied GoalStatus = iota
	// GoalPlanned means a feasible action sequence was found.
	GoalPlanned
	// GoalUnreachable means no sequence of available actions can achieve
	// the goal from the current state within the search limits.
	GoalUnreachable
	// GoalConflicting means the goal's desired facts contradict facts
	// required by a higher-priority goal that was already committed.
	GoalConflicting
	// GoalDeferred means a dependency is not (yet) satisfiable, so the
	// goal was skipped in this round.
	GoalDeferred
)

func (s GoalStatus) String() string {
	switch s {
	case GoalSatisfied:
		return "satisfied"
	case GoalPlanned:
		return "planned"
	case GoalUnreachable:
		return "unreachable"
	case GoalConflicting:
		return "conflicting"
	case GoalDeferred:
		return "deferred"
	}
	return "unknown"
}

// GoalResult pairs a goal with its evaluation outcome.
type GoalResult struct {
	Goal   *Goal
	Status GoalStatus
	Plan   *Plan // non-nil only when Status == GoalPlanned
}

// orderGoals returns goals sorted deterministically for selection:
// higher priority first, ties broken by name. The input is not mutated.
func orderGoals(goals []*Goal) []*Goal {
	out := make([]*Goal, len(goals))
	copy(out, goals)
	// insertion sort on (priority desc, name asc) — stable and simple
	for i := 1; i < len(out); i++ {
		for j := i; j > 0; j-- {
			a, b := out[j-1], out[j]
			if a.Priority > b.Priority ||
				(a.Priority == b.Priority && a.Name <= b.Name) {
				break
			}
			out[j-1], out[j] = out[j], out[j-1]
		}
	}
	return out
}
