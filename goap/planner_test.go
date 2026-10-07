package goap

import (
	"reflect"
	"testing"
)

func TestPlanSimpleChain(t *testing.T) {
	actions := NewActionSet(
		&Action{Name: "chop", Cost: 1, Effects: []Effect{{Key: "wood", Value: true}}},
		&Action{Name: "build", Cost: 2, Preconditions: []Condition{{Key: "wood", Value: true}}, Effects: []Effect{{Key: "house", Value: true}}},
	)
	p := NewPlanner(actions, PlannerConfig{})
	plan, err := p.Plan(State{}, []Condition{{Key: "house", Value: true}})
	if err != nil {
		t.Fatalf("unexpected error: %v", err)
	}
	if got, want := plan.Names(), []string{"chop", "build"}; !reflect.DeepEqual(got, want) {
		t.Fatalf("got %v want %v", got, want)
	}
	if plan.Cost != 3 {
		t.Fatalf("cost = %v, want 3", plan.Cost)
	}
}

func TestPlanEquivalentPathsTieBreak(t *testing.T) {
	// Two equal-cost paths to the goal: ["zeta"] and ["alpha"].
	// The lexicographically smaller action-name sequence must win,
	// regardless of the order actions are registered.
	mk := func() *Planner {
		return NewPlanner(NewActionSet(
			&Action{Name: "zeta", Cost: 1, Effects: []Effect{{Key: "done", Value: true}}},
			&Action{Name: "alpha", Cost: 1, Effects: []Effect{{Key: "done", Value: true}}},
		), PlannerConfig{})
	}
	goal := []Condition{{Key: "done", Value: true}}
	plan, err := mk().Plan(State{}, goal)
	if err != nil {
		t.Fatal(err)
	}
	if got := plan.Names(); !reflect.DeepEqual(got, []string{"alpha"}) {
		t.Fatalf("tie-break violated: got %v want [alpha]", got)
	}
	// Determinism across repeated runs.
	for i := 0; i < 20; i++ {
		pl, err := mk().Plan(State{}, goal)
		if err != nil || !reflect.DeepEqual(pl.Names(), plan.Names()) {
			t.Fatalf("run %d not deterministic: %v %v", i, pl, err)
		}
	}
}

func TestPlanCheaperBeatsLexicographic(t *testing.T) {
	// "aaa" costs 5; "zzz" costs 1. Cost dominates the tie-break.
	p := NewPlanner(NewActionSet(
		&Action{Name: "aaa", Cost: 5, Effects: []Effect{{Key: "done", Value: true}}},
		&Action{Name: "zzz", Cost: 1, Effects: []Effect{{Key: "done", Value: true}}},
	), PlannerConfig{})
	plan, err := p.Plan(State{}, []Condition{{Key: "done", Value: true}})
	if err != nil {
		t.Fatal(err)
	}
	if got := plan.Names(); !reflect.DeepEqual(got, []string{"zzz"}) {
		t.Fatalf("got %v want [zzz]", got)
	}
}

func TestPlanMutexConstraint(t *testing.T) {
	// Both actions are needed in spirit, but they share a mutex group,
	// so the goal requiring both effects is unreachable.
	p := NewPlanner(NewActionSet(
		&Action{Name: "a", Cost: 1, Mutex: []string{"tool"}, Effects: []Effect{{Key: "x", Value: true}}},
		&Action{Name: "b", Cost: 1, Mutex: []string{"tool"}, Effects: []Effect{{Key: "y", Value: true}}},
	), PlannerConfig{})
	_, err := p.Plan(State{}, []Condition{{Key: "x", Value: true}, {Key: "y", Value: true}})
	if err != ErrNoPlan {
		t.Fatalf("got %v want ErrNoPlan", err)
	}
	// Each individual goal is reachable.
	plan, err := p.Plan(State{}, []Condition{{Key: "x", Value: true}})
	if err != nil || !reflect.DeepEqual(plan.Names(), []string{"a"}) {
		t.Fatalf("got %v %v", plan, err)
	}
}

func TestPlanUnreachableAndSearchLimit(t *testing.T) {
	p := NewPlanner(NewActionSet(
		&Action{Name: "noop", Cost: 1, Effects: []Effect{{Key: "n", Value: 1}}},
	), PlannerConfig{})
	if _, err := p.Plan(State{}, []Condition{{Key: "impossible", Value: true}}); err != ErrNoPlan {
		t.Fatalf("got %v want ErrNoPlan", err)
	}
	// A cycle: increment-only action can never reach the goal, and the
	// search must terminate via the iteration budget, not hang.
	big := NewPlanner(NewActionSet(
		&Action{Name: "spin", Cost: 1,
			Preconditions: []Condition{{Key: "n", Value: 1}},
			Effects:       []Effect{{Key: "n", Value: 1}}},
	), PlannerConfig{MaxIterations: 50, MaxDepth: 10})
	if _, err := big.Plan(State{"n": 1}, []Condition{{Key: "n", Value: 999}}); err != ErrNoPlan && err != ErrSearchLimit {
		t.Fatalf("got %v, want ErrNoPlan or ErrSearchLimit", err)
	}
}

func TestPlanGoalsPriorityConflictAndDeps(t *testing.T) {
	actions := NewActionSet(
		&Action{Name: "heat", Cost: 1, Effects: []Effect{{Key: "room", Value: "warm"}}},
		&Action{Name: "cool", Cost: 1, Effects: []Effect{{Key: "room", Value: "cold"}}},
		&Action{Name: "bake", Cost: 1, Preconditions: []Condition{{Key: "room", Value: "warm"}}, Effects: []Effect{{Key: "cake", Value: true}}},
	)
	p := NewPlanner(actions, PlannerConfig{})
	goals := []*Goal{
		{Name: "warm-room", Priority: 10, Desired: []Condition{{Key: "room", Value: "warm"}}},
		{Name: "cold-room", Priority: 5, Desired: []Condition{{Key: "room", Value: "cold"}}},
		{Name: "cake", Priority: 7, Desired: []Condition{{Key: "cake", Value: true}}, DependsOn: []string{"warm-room"}},
		{Name: "impossible", Priority: 1, Desired: []Condition{{Key: "moon", Value: true}}},
	}
	results := p.PlanGoals(State{}, goals)
	got := map[string]GoalStatus{}
	for _, r := range results {
		got[r.Goal.Name] = r.Status
	}
	want := map[string]GoalStatus{
		"warm-room":  GoalPlanned,
		"cold-room":  GoalConflicting, // contradicts committed warm-room
		"cake":       GoalPlanned,     // dependency planned, preconditions met
		"impossible": GoalUnreachable,
	}
	if !reflect.DeepEqual(got, want) {
		t.Fatalf("got %v want %v", got, want)
	}
	// Deterministic evaluation order: priority desc, then name.
	order := []string{}
	for _, r := range results {
		order = append(order, r.Goal.Name)
	}
	if want := []string{"warm-room", "cake", "cold-room", "impossible"}; !reflect.DeepEqual(order, want) {
		t.Fatalf("order got %v want %v", order, want)
	}
}

func TestPlanGoalsDeferredDependency(t *testing.T) {
	p := NewPlanner(NewActionSet(), PlannerConfig{})
	goals := []*Goal{
		{Name: "child", Priority: 10, Desired: []Condition{{Key: "c", Value: true}}, DependsOn: []string{"parent"}},
		{Name: "parent", Priority: 1, Desired: []Condition{{Key: "p", Value: true}}}, // unreachable: no actions
	}
	results := p.PlanGoals(State{}, goals)
	got := map[string]GoalStatus{}
	for _, r := range results {
		got[r.Goal.Name] = r.Status
	}
	if got["parent"] != GoalUnreachable || got["child"] != GoalDeferred {
		t.Fatalf("got %v", got)
	}
}

func TestPlanGoalsMerging(t *testing.T) {
	// g2's plan must build on the state after g1's plan: "step2" needs
	// the fact produced by g1's action.
	actions := NewActionSet(
		&Action{Name: "one", Cost: 1, Effects: []Effect{{Key: "a", Value: true}}},
		&Action{Name: "two", Cost: 1, Preconditions: []Condition{{Key: "a", Value: true}}, Effects: []Effect{{Key: "b", Value: true}}},
	)
	p := NewPlanner(actions, PlannerConfig{})
	results := p.PlanGoals(State{}, []*Goal{
		{Name: "g1", Priority: 2, Desired: []Condition{{Key: "a", Value: true}}},
		{Name: "g2", Priority: 1, Desired: []Condition{{Key: "b", Value: true}}},
	})
	if results[0].Status != GoalPlanned || results[1].Status != GoalPlanned {
		t.Fatalf("statuses: %v %v", results[0].Status, results[1].Status)
	}
	merged := append(results[0].Plan.Names(), results[1].Plan.Names()...)
	if !reflect.DeepEqual(merged, []string{"one", "two"}) {
		t.Fatalf("merged plan = %v", merged)
	}
}
