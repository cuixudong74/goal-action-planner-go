package goap

import (
	"context"
	"reflect"
	"strings"
	"testing"
	"time"
)

// contentionActions: both agents want the single "crane" resource to
// build their own house.
func contentionActions() *ActionSet {
	return NewActionSet(
		&Action{Name: "build-a", Cost: 1, Resources: []string{"crane"},
			Effects: []Effect{{Key: "house-a", Value: true}}},
		&Action{Name: "build-b", Cost: 1, Resources: []string{"crane"},
			Effects: []Effect{{Key: "house-b", Value: true}}},
	)
}

func TestRuntimeResourceContentionDeterministic(t *testing.T) {
	run := func() (State, []Event) {
		w := NewWorld(State{})
		rt := NewRuntime(w, NewPlanner(contentionActions(), PlannerConfig{}), 42, RuntimeConfig{})
		mk := func(id, key string) Agent {
			return AgentFunc{Name: id, Fn: func(ctx context.Context, h *Handle, snap State, tick uint64) {
				h.RunGoal(ctx, []Condition{{Key: key, Value: true}}, nil)
			}}
		}
		rt.AddAgent(mk("agent-a", "house-a"))
		rt.AddAgent(mk("agent-b", "house-b"))
		events := rt.Run(context.Background(), 2)
		snap, _ := w.Snapshot()
		return snap, events
	}
	s1, e1 := run()
	s2, e2 := run()
	// Frame-identical across identical runs.
	if !s1.Equal(s2) || !reflect.DeepEqual(e1, e2) {
		t.Fatalf("non-deterministic runtime:\n%v / %v\n%v / %v", s1, s2, e1, e2)
	}
	// Both houses eventually stand (two ticks were enough).
	if !s1.Satisfies([]Condition{{Key: "house-a", Value: true}, {Key: "house-b", Value: true}}) {
		t.Fatalf("final state = %v", s1)
	}
	// Tick 1: agent-a acts first (lexicographic order) and wins the
	// crane; agent-b's denial is logged with the holder's identity.
	var denial *Event
	for i := range e1 {
		if e1[i].Kind == EventResourceWait || (e1[i].Kind == EventActionFailed && strings.Contains(e1[i].Detail, "crane")) {
			d := e1[i]
			denial = &d
			break
		}
	}
	if denial == nil {
		t.Fatalf("no contention event logged: %v", e1)
	}
	if denial.Agent != "agent-b" || !strings.Contains(denial.Detail, "agent-a") {
		t.Fatalf("contention not explainable: %+v", denial)
	}
}

func TestRuntimeSlowAgentDoesNotBlock(t *testing.T) {
	w := NewWorld(State{})
	rt := NewRuntime(w, NewPlanner(execActions(), PlannerConfig{}), 7,
		RuntimeConfig{TurnTimeout: 20 * time.Millisecond})
	slow := AgentFunc{Name: "a-slow", Fn: func(ctx context.Context, h *Handle, snap State, tick uint64) {
		select {
		case <-time.After(time.Hour):
		case <-ctx.Done(): // a well-behaved slow agent still gets cut off
		}
	}}
	fast := AgentFunc{Name: "b-fast", Fn: func(ctx context.Context, h *Handle, snap State, tick uint64) {
		h.RunGoal(ctx, houseGoal(), nil)
	}}
	rt.AddAgent(slow)
	rt.AddAgent(fast)
	start := time.Now()
	rt.Run(context.Background(), 3)
	elapsed := time.Since(start)
	if s, _ := w.Snapshot(); !s.Satisfies(houseGoal()) {
		t.Fatalf("fast agent was blocked by slow agent: %v", s)
	}
	// 3 ticks * 20ms timeout bound the slow agent's total impact.
	if elapsed > 5*time.Second {
		t.Fatalf("slow agent blocked runtime for %v", elapsed)
	}
}

func TestRuntimeInterruptResumeAlignment(t *testing.T) {
	actions := NewActionSet(
		&Action{Name: "s1", Cost: 1, Effects: []Effect{{Key: "k1", Value: true}}},
		&Action{Name: "s2", Cost: 1, Preconditions: []Condition{{Key: "k1", Value: true}}, Effects: []Effect{{Key: "k2", Value: true}}},
		&Action{Name: "s3", Cost: 1, Preconditions: []Condition{{Key: "k2", Value: true}}, Effects: []Effect{{Key: "k3", Value: true}}},
	)
	goal := []Condition{{Key: "k3", Value: true}}

	// Uninterrupted reference run.
	refWorld := NewWorld(State{})
	refRt := NewRuntime(refWorld, NewPlanner(actions, PlannerConfig{}), 1, RuntimeConfig{})
	refRt.AddAgent(AgentFunc{Name: "agent", Fn: func(ctx context.Context, h *Handle, snap State, tick uint64) {
		h.RunGoal(ctx, goal, nil)
	}})
	refRt.Run(context.Background(), 1)
	refSnap, _ := refWorld.Snapshot()

	// Interrupted run: cancel after the first action lands, then resume.
	intWorld := NewWorld(State{})
	ctx, cancel := context.WithCancel(context.Background())
	intRt := NewRuntime(intWorld, NewPlanner(actions, PlannerConfig{}), 1, RuntimeConfig{})
	steps := 0
	intRt.AddAgent(AgentFunc{Name: "agent", Fn: func(ctx context.Context, h *Handle, snap State, tick uint64) {
		if tick == 1 {
			// Cancel the turn's context after the first atomic step by
			// wrapping: run one action, then cancel.
			h.RunGoal(ctx, []Condition{{Key: "k1", Value: true}}, nil)
			steps++
			cancel()
			return
		}
		h.RunGoal(ctx, goal, nil)
	}})
	intRt.Run(ctx, 1) // first tick, cancelled partway
	// No dangling resource holds and no half-applied actions: state
	// must exactly satisfy k1 and nothing more.
	mid, _ := intWorld.Snapshot()
	if !mid.Satisfies([]Condition{{Key: "k1", Value: true}}) || mid["k2"] != nil {
		t.Fatalf("partial state after interrupt: %v", mid)
	}
	// Resume with a fresh context.
	intRt.Run(context.Background(), 1)
	finalSnap, _ := intWorld.Snapshot()
	if !finalSnap.Equal(refSnap) {
		t.Fatalf("resume diverged: got %v want %v", finalSnap, refSnap)
	}
}

func TestRuntimeDeterminismAcrossRuns(t *testing.T) {
	run := func() ([]Event, State) {
		w := NewWorld(State{})
		rt := NewRuntime(w, NewPlanner(execActions(), PlannerConfig{}), 99, RuntimeConfig{})
		for _, id := range []string{"c", "a", "b"} { // unsorted on purpose
			id := id
			rt.AddAgent(AgentFunc{Name: id, Fn: func(ctx context.Context, h *Handle, snap State, tick uint64) {
				_ = h.Rand() // consume deterministic randomness
				if id == "a" {
					h.RunGoal(ctx, houseGoal(), nil)
				}
			}})
		}
		ev := rt.Run(context.Background(), 2)
		s, _ := w.Snapshot()
		return ev, s
	}
	e1, s1 := run()
	e2, s2 := run()
	if !reflect.DeepEqual(e1, e2) || !s1.Equal(s2) {
		t.Fatalf("non-deterministic: %v vs %v", e1, e2)
	}
}
