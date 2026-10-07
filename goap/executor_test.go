package goap

import (
	"context"
	"errors"
	"reflect"
	"testing"
)

func execActions() *ActionSet {
	return NewActionSet(
		&Action{Name: "chop", Cost: 1, Effects: []Effect{{Key: "wood", Value: true}}},
		&Action{Name: "build", Cost: 1, Preconditions: []Condition{{Key: "wood", Value: true}}, Effects: []Effect{{Key: "house", Value: true}}},
	)
}

func houseGoal() []Condition { return []Condition{{Key: "house", Value: true}} }

func TestExecutorHappyPath(t *testing.T) {
	w := NewWorld(State{})
	ex := NewExecutor(NewPlanner(execActions(), PlannerConfig{}), nil, ExecutorConfig{})
	res := ex.Run(context.Background(), "a1", w, houseGoal(), nil)
	if res.Outcome != OutcomeSuccess {
		t.Fatalf("outcome = %v", res.Outcome)
	}
	if !reflect.DeepEqual(res.Executed, []string{"chop", "build"}) {
		t.Fatalf("executed = %v", res.Executed)
	}
	if s, _ := w.Snapshot(); !s.Satisfies(houseGoal()) {
		t.Fatalf("goal not reached: %v", s)
	}
}

func TestExecutorSkipsAlreadyEffectiveActions(t *testing.T) {
	// "wood" is already true: chop must not be re-executed.
	w := NewWorld(State{"wood": true})
	ex := NewExecutor(NewPlanner(execActions(), PlannerConfig{}), nil, ExecutorConfig{})
	res := ex.Run(context.Background(), "a1", w, houseGoal(), nil)
	if res.Outcome != OutcomeSuccess {
		t.Fatalf("outcome = %v", res.Outcome)
	}
	if !reflect.DeepEqual(res.Executed, []string{"build"}) {
		t.Fatalf("executed = %v, want [build]", res.Executed)
	}
}

func TestExecutorReplanOnEnvironmentalDrift(t *testing.T) {
	// The environment flips "wood" back to false after chop lands,
	// simulating drift. The executor must detect the broken precondition
	// of "build" and re-plan from the interruption point; chop runs
	// exactly twice in total and build succeeds.
	w := NewWorld(State{})
	drifted := false
	runner := RunnerFunc(func(ctx context.Context, w *World, a *Action, base uint64) (uint64, error) {
		v, err := w.ApplyIfVersion(a.Effects, base)
		if err != nil {
			return v, err
		}
		if a.Name == "chop" && !drifted {
			drifted = true
			w.Apply([]Effect{{Key: "wood", Value: false}}) // environmental drift
		}
		return v, nil
	})
	ex := NewExecutor(NewPlanner(execActions(), PlannerConfig{}), runner, ExecutorConfig{})
	res := ex.Run(context.Background(), "a1", w, houseGoal(), nil)
	if res.Outcome != OutcomeSuccess {
		t.Fatalf("outcome = %v", res.Outcome)
	}
	// The first chop's effects were wiped by drift, so only the chop
	// whose effects survived is recorded; chop is not double-counted.
	if !reflect.DeepEqual(res.Executed, []string{"chop", "build"}) {
		t.Fatalf("executed = %v", res.Executed)
	}
	// The drift must be detected (failed effect verification) and a
	// re-plan must be recorded.
	var failed, replan bool
	var chopStarts int
	for _, ev := range res.Events {
		failed = failed || ev.Kind == EventActionFailed
		replan = replan || ev.Kind == EventReplan
		if ev.Kind == EventActionStarted && ev.Action == "chop" {
			chopStarts++
		}
	}
	if !failed || !replan {
		t.Fatalf("expected failure+replan events, got %v", res.Events)
	}
	if chopStarts != 2 {
		t.Fatalf("chop started %d times, want 2 (retry after drift)", chopStarts)
	}
}

func TestExecutorReplanOnActionFailure(t *testing.T) {
	// "build" fails once (transient), then succeeds. Recovery must not
	// re-execute "chop" whose effects are already in place.
	w := NewWorld(State{})
	failed := false
	runner := RunnerFunc(func(ctx context.Context, w *World, a *Action, base uint64) (uint64, error) {
		if a.Name == "build" && !failed {
			failed = true
			return base, errors.New("simulated failure")
		}
		return w.ApplyIfVersion(a.Effects, base)
	})
	ex := NewExecutor(NewPlanner(execActions(), PlannerConfig{}), runner, ExecutorConfig{})
	res := ex.Run(context.Background(), "a1", w, houseGoal(), nil)
	if res.Outcome != OutcomeSuccess {
		t.Fatalf("outcome = %v", res.Outcome)
	}
	if !reflect.DeepEqual(res.Executed, []string{"chop", "build"}) {
		t.Fatalf("executed = %v, want [chop build] (no duplicate chop)", res.Executed)
	}
}

func TestExecutorVersionConflictTriggersReplan(t *testing.T) {
	// A concurrent writer bumps the world version between validation
	// and apply; the CAS fails and the executor re-plans.
	w := NewWorld(State{})
	once := false
	runner := RunnerFunc(func(ctx context.Context, w *World, a *Action, base uint64) (uint64, error) {
		if a.Name == "chop" && !once {
			once = true
			w.Apply([]Effect{{Key: "noise", Value: true}}) // concurrent change
		}
		return w.ApplyIfVersion(a.Effects, base)
	})
	ex := NewExecutor(NewPlanner(execActions(), PlannerConfig{}), runner, ExecutorConfig{})
	res := ex.Run(context.Background(), "a1", w, houseGoal(), nil)
	if res.Outcome != OutcomeSuccess {
		t.Fatalf("outcome = %v", res.Outcome)
	}
	if !reflect.DeepEqual(res.Executed, []string{"chop", "build"}) {
		t.Fatalf("executed = %v", res.Executed)
	}
}

func TestExecutorNoPlan(t *testing.T) {
	w := NewWorld(State{})
	ex := NewExecutor(NewPlanner(execActions(), PlannerConfig{}), nil, ExecutorConfig{})
	res := ex.Run(context.Background(), "a1", w, []Condition{{Key: "moon", Value: true}}, nil)
	if res.Outcome != OutcomeNoPlan {
		t.Fatalf("outcome = %v", res.Outcome)
	}
}

func TestExecutorCancel(t *testing.T) {
	w := NewWorld(State{})
	ctx, cancel := context.WithCancel(context.Background())
	runner := RunnerFunc(func(ctx context.Context, w *World, a *Action, base uint64) (uint64, error) {
		v, err := w.ApplyIfVersion(a.Effects, base)
		cancel() // cancel after the first atomic step
		return v, err
	})
	ex := NewExecutor(NewPlanner(execActions(), PlannerConfig{}), runner, ExecutorConfig{})
	res := ex.Run(ctx, "a1", w, houseGoal(), nil)
	if res.Outcome != OutcomeCanceled {
		t.Fatalf("outcome = %v", res.Outcome)
	}
	// Only chop applied; nothing half-executed.
	if !reflect.DeepEqual(res.Executed, []string{"chop"}) {
		t.Fatalf("executed = %v", res.Executed)
	}
	// Resume with a fresh context: converges to the same end state as
	// an uninterrupted run, without re-executing chop.
	res2 := ex.Run(context.Background(), "a1", w, houseGoal(), nil)
	if res2.Outcome != OutcomeSuccess || !reflect.DeepEqual(res2.Executed, []string{"build"}) {
		t.Fatalf("resume: %v %v", res2.Outcome, res2.Executed)
	}
	if s, _ := w.Snapshot(); !s.Satisfies(houseGoal()) {
		t.Fatalf("goal not reached after resume: %v", s)
	}
}
