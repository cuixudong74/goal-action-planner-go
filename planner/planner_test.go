package planner

import (
	"testing"

	"github.com/cuixudong74/goal-action-planner-go/world"
)

func eq(key string, v int64) world.Condition {
	return world.Condition{Key: key, Op: world.OpEq, Value: v}
}

func set(key string, v int64) world.Effect {
	return world.Effect{Key: key, Op: world.EffSet, Value: v}
}

// Two paths reach the goal at equal cost and equal length; the
// lexicographically smaller action-name sequence must win, regardless of
// the order actions are registered in.
func TestEquivalentPathsDeterministicTieBreak(t *testing.T) {
	mk := func() []*Action {
		return []*Action{
			{Name: "b-step", Cost: 1, Effects: []world.Effect{set("x", 1)}},
			{Name: "a-step", Cost: 1, Effects: []world.Effect{set("x", 1)}},
		}
	}
	goal := []Goal{{Name: "g", Priority: 1, Conditions: []world.Condition{eq("x", 1)}}}
	st := world.NewState(nil)

	// Register actions in both orders; the result must be identical.
	p1 := New(mk(), Options{})
	p2 := New([]*Action{mk()[1], mk()[0]}, Options{})

	r1 := p1.Plan(st, 0, goal)
	r2 := p2.Plan(st, 0, goal)

	if r1.Status != StatusOK || r2.Status != StatusOK {
		t.Fatalf("expected ok, got %v / %v", r1.Status, r2.Status)
	}
	if got := r1.Plan.Names(); len(got) != 1 || got[0] != "a-step" {
		t.Fatalf("tie-break should pick a-step, got %v", got)
	}
	if r1.Plan.Names()[0] != r2.Plan.Names()[0] {
		t.Fatalf("registration order leaked into result: %v vs %v", r1.Plan.Names(), r2.Plan.Names())
	}
}

// Lower cost must beat shorter-but-pricier alternatives.
func TestCostBeatsLength(t *testing.T) {
	actions := []*Action{
		{Name: "direct", Cost: 5, Effects: []world.Effect{set("x", 1)}},
		{Name: "hop1", Cost: 1, Effects: []world.Effect{set("y", 1)}},
		{Name: "hop2", Cost: 1, Preconditions: []world.Condition{eq("y", 1)}, Effects: []world.Effect{set("x", 1)}},
	}
	p := New(actions, Options{})
	r := p.Plan(world.NewState(nil), 0, []Goal{{Name: "g", Conditions: []world.Condition{eq("x", 1)}}})
	if r.Status != StatusOK {
		t.Fatalf("status %v", r.Status)
	}
	want := []string{"hop1", "hop2"}
	got := r.Plan.Names()
	if len(got) != 2 || got[0] != want[0] || got[1] != want[1] {
		t.Fatalf("want %v got %v (cost %v)", want, got, r.Plan.TotalCost)
	}
}

// Contradictory goals must be reported as a distinguishable conflict.
func TestGoalConflict(t *testing.T) {
	p := New(nil, Options{})
	r := p.Plan(world.NewState(nil), 0, []Goal{
		{Name: "low", Priority: 1, Conditions: []world.Condition{eq("temp", 3)}},
		{Name: "high", Priority: 2, Conditions: []world.Condition{eq("temp", 9)}},
	})
	if r.Status != StatusGoalConflict {
		t.Fatalf("status %v", r.Status)
	}
	if len(r.Conflicts) != 1 || r.Conflicts[0].Key != "temp" {
		t.Fatalf("conflicts %+v", r.Conflicts)
	}
}

// Range-vs-equality contradiction must also be detected.
func TestRangeConflict(t *testing.T) {
	p := New(nil, Options{})
	r := p.Plan(world.NewState(nil), 0, []Goal{
		{Name: "a", Conditions: []world.Condition{{Key: "n", Op: world.OpGt, Value: 10}}},
		{Name: "b", Conditions: []world.Condition{{Key: "n", Op: world.OpLt, Value: 5}}},
	})
	if r.Status != StatusGoalConflict {
		t.Fatalf("status %v", r.Status)
	}
}

// A goal no action can reach must be reported as unreachable, with the
// offending goal named, and must not hang.
func TestUnreachableGoal(t *testing.T) {
	actions := []*Action{
		{Name: "work", Cost: 1, Effects: []world.Effect{set("done", 1)}},
	}
	p := New(actions, Options{})
	r := p.Plan(world.NewState(nil), 0, []Goal{
		{Name: "possible", Conditions: []world.Condition{eq("done", 1)}},
		{Name: "impossible", Conditions: []world.Condition{eq("moon", 1)}},
	})
	if r.Status != StatusUnreachable {
		t.Fatalf("status %v", r.Status)
	}
	if len(r.UnreachableGoals) != 1 || r.UnreachableGoals[0] != "impossible" {
		t.Fatalf("unreachable %v", r.UnreachableGoals)
	}
}

// Dependency cycles must be reported, not followed forever.
func TestDependencyCycle(t *testing.T) {
	p := New(nil, Options{})
	r := p.Plan(world.NewState(nil), 0, []Goal{
		{Name: "a", DependsOn: []string{"b"}},
		{Name: "b", DependsOn: []string{"a"}},
	})
	if r.Status != StatusDependencyCycle {
		t.Fatalf("status %v", r.Status)
	}
	if len(r.Cycle) == 0 {
		t.Fatal("expected cycle path")
	}
}

// A goal whose dependency is absent from the goal set is ineligible and
// silently dropped from the merged set (only eligible goals are planned).
func TestMissingDependencyDropsGoal(t *testing.T) {
	p := New(nil, Options{})
	r := p.Plan(world.NewState(nil), 0, []Goal{
		{Name: "standalone", Conditions: []world.Condition{eq("a", 1)}},
		{Name: "needy", DependsOn: []string{"ghost"}, Conditions: []world.Condition{eq("b", 1)}},
	})
	if r.Status != StatusUnreachable { // no actions at all, "a" unreachable
		t.Fatalf("status %v", r.Status)
	}
	for _, g := range r.Goals {
		if g == "needy" {
			t.Fatalf("needy should have been dropped: %v", r.Goals)
		}
	}
}

// Priority ordering must decide the merged goal set order deterministically.
func TestGoalOrderingByPriorityThenName(t *testing.T) {
	goals := orderGoals([]Goal{
		{Name: "z", Priority: 1},
		{Name: "a", Priority: 5},
		{Name: "m", Priority: 5},
	})
	want := []string{"a", "m", "z"}
	for i, g := range goals {
		if g.Name != want[i] {
			t.Fatalf("order %v, want %v", goalNames(goals), want)
		}
	}
}

// Plan-level mutual exclusion: two actions sharing a MutexKey must never
// appear in the same plan.
func TestMutexKeyExcludesCooccurrence(t *testing.T) {
	actions := []*Action{
		{Name: "tool-a-grab", Cost: 1, MutexKey: "hand", Effects: []world.Effect{set("held-a", 1)}},
		{Name: "tool-b-grab", Cost: 1, MutexKey: "hand", Effects: []world.Effect{set("held-b", 1)}},
	}
	p := New(actions, Options{})
	r := p.Plan(world.NewState(nil), 0, []Goal{
		{Name: "both", Conditions: []world.Condition{eq("held-a", 1), eq("held-b", 1)}},
	})
	if r.Status != StatusUnreachable {
		t.Fatalf("mutex should make joint goal unreachable, got %v (%v)", r.Status, r.Plan.Names())
	}
}

// The search must terminate with a budget error instead of hanging when
// the state space is effectively unbounded.
func TestBudgetExhaustion(t *testing.T) {
	actions := []*Action{
		{Name: "inc", Cost: 1, Effects: []world.Effect{{Key: "n", Op: world.EffAdd, Value: 1}}},
	}
	p := New(actions, Options{MaxExpansions: 100, MaxDepth: 1000})
	r := p.Plan(world.NewState(nil), 0, []Goal{
		{Name: "far", Conditions: []world.Condition{eq("n", 1<<40)}},
	})
	if r.Status != StatusBudgetExhausted {
		t.Fatalf("status %v", r.Status)
	}
}

// Identical inputs must produce byte-identical plans across repeated runs.
func TestDeterminismAcrossRuns(t *testing.T) {
	mk := func() *Planner {
		return New([]*Action{
			{Name: "c", Cost: 1, Effects: []world.Effect{set("z", 1)}},
			{Name: "a", Cost: 1, Effects: []world.Effect{set("x", 1)}},
			{Name: "b", Cost: 1, Preconditions: []world.Condition{eq("x", 1)}, Effects: []world.Effect{set("y", 1)}},
			{Name: "d", Cost: 1, Preconditions: []world.Condition{eq("y", 1)}, Effects: []world.Effect{set("z", 1)}},
		}, Options{})
	}
	goal := []Goal{{Name: "g", Conditions: []world.Condition{eq("z", 1)}}}
	first := mk().Plan(world.NewState(nil), 7, goal)
	for i := 0; i < 20; i++ {
		r := mk().Plan(world.NewState(nil), 7, goal)
		if r.Status != first.Status || r.Plan.TotalCost != first.Plan.TotalCost {
			t.Fatalf("run %d diverged", i)
		}
		if seqString(r.Plan.Names()) != seqString(first.Plan.Names()) {
			t.Fatalf("run %d diverged: %v vs %v", i, r.Plan.Names(), first.Plan.Names())
		}
	}
	// cheapest path to z=1 is the single action "c" (cost 1)
	if got := first.Plan.Names(); len(got) != 1 || got[0] != "c" {
		t.Fatalf("unexpected plan %v", got)
	}
	if first.Plan.BaseVersion != 7 {
		t.Fatalf("base version %d", first.Plan.BaseVersion)
	}
}
