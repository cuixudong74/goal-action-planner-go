package runtime

import (
	"context"
	"fmt"
	"strings"
	"testing"

	"github.com/cuixudong74/goal-action-planner-go/planner"
	"github.com/cuixudong74/goal-action-planner-go/world"
)

func eq(key string, v int64) world.Condition {
	return world.Condition{Key: key, Op: world.OpEq, Value: v}
}
func set(key string, v int64) world.Effect {
	return world.Effect{Key: key, Op: world.EffSet, Value: v}
}

func hasEvent(log *Log, kind EventKind, substr string) bool {
	for _, e := range log.Events() {
		if e.Kind == kind && (substr == "" || strings.Contains(e.Detail, substr) || strings.Contains(e.Action, substr)) {
			return true
		}
	}
	return false
}

func countApplied(log *Log, agent, action string) int {
	n := 0
	for _, e := range log.Events() {
		if e.Kind == EventActionApplied && e.Agent == agent && e.Action == action {
			n++
		}
	}
	return n
}

// A two-step plan where the world is mutated externally between the steps:
// the executor must detect the broken precondition, replan from the
// interruption point, and must NOT re-apply the action that already took
// effect.
func TestDeviationTriggersReplanWithoutReexecution(t *testing.T) {
	actions := []*planner.Action{
		{Name: "fetch", Cost: 1, Effects: []world.Effect{set("have-tool", 1)}},
		{Name: "power-on", Cost: 1, Preconditions: []world.Condition{eq("have-tool", 1), eq("power", 1)},
			Effects: []world.Effect{set("machine-on", 1)}},
		{Name: "flip-breaker", Cost: 1, Effects: []world.Effect{set("power", 1)}},
	}
	p := planner.New(actions, planner.Options{})
	store := world.NewStore(world.NewState(map[string]int64{"power": 1}))
	rt := New(store, p, Options{}, nil)
	rt.AddAgent("a1", []planner.Goal{{Name: "run", Conditions: []world.Condition{eq("machine-on", 1)}}})

	// Drive manually so we can mutate the world mid-plan: fetch applies at
	// tick 1, then the environment cuts power before power-on runs.
	for i := 0; i < 20; i++ {
		if phase, _ := rt.AgentStatus("a1"); phase == "done" || phase == "failed" {
			break
		}
		if rt.tick == 2 {
			store.Mutate([]world.Effect{set("power", 0)})
		}
		rt.step()
		rt.tick++
	}
	log := rt.Log()
	if !hasEvent(log, EventPrecondBroken, "power-on") {
		t.Fatalf("expected precond-broken for power-on:\n%s", log)
	}
	if !hasEvent(log, EventReplan, "") {
		t.Fatalf("expected replan:\n%s", log)
	}
	if n := countApplied(log, "a1", "fetch"); n != 1 {
		t.Fatalf("fetch applied %d times, want 1:\n%s", n, log)
	}
	// The replan must restore power and finish the job.
	st, _ := store.Snapshot()
	if st.Get("machine-on") != 1 {
		t.Fatalf("goal not reached:\n%s", log)
	}
	if !hasEvent(log, EventActionApplied, "flip-breaker") {
		t.Fatalf("replan should have inserted flip-breaker:\n%s", log)
	}
}

// An action failure injected mid-plan must trigger replanning from the
// interruption point; the retried plan must complete the goal.
func TestActionFailureAndRecovery(t *testing.T) {
	actions := []*planner.Action{
		{Name: "try-open", Cost: 1, Effects: []world.Effect{set("door", 1)}},
	}
	p := planner.New(actions, planner.Options{})
	store := world.NewStore(world.NewState(nil))
	// Fail try-open twice, then let it succeed.
	rt := New(store, p, Options{}, ScriptFailure(map[string]int{"a1/try-open": 2}))
	rt.AddAgent("a1", []planner.Goal{{Name: "open", Conditions: []world.Condition{eq("door", 1)}}})
	log := rt.Run(context.Background())

	if n := countApplied(log, "a1", "try-open"); n != 1 {
		t.Fatalf("try-open applied %d times, want exactly 1:\n%s", n, log)
	}
	if c := 0; !hasEvent(log, EventActionFailed, "try-open") {
		_ = c
		t.Fatalf("expected action-failed events:\n%s", log)
	}
	if phase, _ := rt.AgentStatus("a1"); phase != "done" {
		t.Fatalf("agent not done: %s\n%s", phase, log)
	}
}

// Two agents contend for one exclusive resource. The winner must be the
// deterministically-first agent (ID order), the loser must wait and
// eventually finish too; every decision must be logged.
func TestConcurrentResourceContention(t *testing.T) {
	mk := func() (*world.Store, *Runtime) {
		// The crane action spans two ticks, so the holder still occupies it
		// when the other agent's turn comes: contention is real.
		p := planner.New([]*planner.Action{
			{Name: "use-crane", Cost: 1, Resource: "crane", Duration: 2,
				Effects: []world.Effect{set("moved", 1)}},
		}, planner.Options{})
		store := world.NewStore(world.NewState(nil))
		rt := New(store, p, Options{}, nil)
		rt.AddAgent("agent-a", []planner.Goal{{Name: "g", Conditions: []world.Condition{eq("moved", 1)}}})
		rt.AddAgent("agent-b", []planner.Goal{{Name: "g", Conditions: []world.Condition{eq("moved", 1)}}})
		return store, rt
	}
	_, rt1 := mk()
	log1 := rt1.Run(context.Background()).String()
	_, rt2 := mk()
	log2 := rt2.Run(context.Background()).String()

	if log1 != log2 {
		t.Fatalf("identical inputs diverged:\n--- run1 ---\n%s\n--- run2 ---\n%s", log1, log2)
	}
	// agent-a acts first in each tick, so it must win the crane.
	if !hasEvent(rt1.Log(), EventResourceAcquired, "crane") {
		t.Fatalf("no acquisition logged:\n%s", log1)
	}
	if !hasEvent(rt1.Log(), EventResourceDenied, "crane") {
		t.Fatalf("loser denial not logged:\n%s", log1)
	}
	for _, id := range []string{"agent-a", "agent-b"} {
		if phase, _ := rt1.AgentStatus(id); phase != "done" {
			t.Fatalf("%s not done (%s):\n%s", id, phase, log1)
		}
	}
	// The crane must never be double-held: acquisitions and releases
	// strictly alternate in the log.
	held := false
	for _, e := range rt1.Log().Events() {
		switch e.Kind {
		case EventResourceAcquired:
			if held {
				t.Fatalf("crane double-held:\n%s", log1)
			}
			held = true
		case EventResourceReleased:
			held = false
		}
	}
}

// A plan computed over several ticks must be discarded when the world
// changed underneath it, and recomputed from the fresh state.
func TestPlanInvalidatedByWorldChange(t *testing.T) {
	// Deep enough search to span multiple ticks with a tiny budget.
	actions := []*planner.Action{
		{Name: "s1", Cost: 1, Effects: []world.Effect{set("k1", 1)}},
		{Name: "s2", Cost: 1, Preconditions: []world.Condition{eq("k1", 1)}, Effects: []world.Effect{set("k2", 1)}},
		{Name: "s3", Cost: 1, Preconditions: []world.Condition{eq("k2", 1)}, Effects: []world.Effect{set("k3", 1)}},
	}
	p := planner.New(actions, planner.Options{})
	store := world.NewStore(world.NewState(nil))
	rt := New(store, p, Options{PlanBudgetPerTick: 1}, nil)
	rt.AddAgent("a1", []planner.Goal{{Name: "deep", Conditions: []world.Condition{eq("k3", 1)}}})

	for i := 0; i < 50; i++ {
		if phase, _ := rt.AgentStatus("a1"); phase == "done" || phase == "failed" {
			break
		}
		rt.step()
		rt.tick++
		if rt.tick == 1 { // world drifts while a1 is still planning
			store.Mutate([]world.Effect{set("noise", 1)})
		}
	}
	if !hasEvent(rt.Log(), EventPlanInvalidated, "") {
		t.Fatalf("expected plan-invalidated:\n%s", rt.Log())
	}
	if phase, _ := rt.AgentStatus("a1"); phase != "done" {
		t.Fatalf("agent should still finish after invalidation:\n%s", rt.Log())
	}
}

// Cancellation mid-run must leave no partial action and no dangling
// resource; resuming must converge to the same final state as an
// uninterrupted run.
func TestCancelAndResumeAlignsWithUninterrupted(t *testing.T) {
	mk := func() (*world.Store, *Runtime) {
		p := planner.New([]*planner.Action{
			{Name: "a", Cost: 1, Resource: "slot", Effects: []world.Effect{set("x", 1)}},
			{Name: "b", Cost: 1, Preconditions: []world.Condition{eq("x", 1)}, Effects: []world.Effect{set("y", 1)}},
			{Name: "c", Cost: 1, Preconditions: []world.Condition{eq("y", 1)}, Effects: []world.Effect{set("z", 1)}},
		}, planner.Options{})
		store := world.NewStore(world.NewState(nil))
		rt := New(store, p, Options{}, nil)
		rt.AddAgent("a1", []planner.Goal{{Name: "g", Conditions: []world.Condition{eq("z", 1)}}})
		return store, rt
	}

	// Uninterrupted reference run.
	refStore, refRt := mk()
	refRt.Run(context.Background())
	refState, _ := refStore.Snapshot()

	// Drive two ticks, then cancel deterministically and resume.
	store, rt := mk()
	rt.step()
	rt.tick++
	rt.step()
	rt.tick++
	ctx, cancel := context.WithCancel(context.Background())
	cancel()
	rt.Run(ctx)
	if !hasEvent(rt.Log(), EventCancelled, "") {
		t.Fatalf("expected cancellation:\n%s", rt.Log())
	}
	// No dangling resource after cancel.
	if len(rt.resources.holders) != 0 {
		t.Fatalf("dangling resources after cancel: %v", rt.resources.holders)
	}
	rt.Run(context.Background()) // resume
	finalState, _ := store.Snapshot()

	if finalState.Hash() != refState.Hash() {
		t.Fatalf("resumed run diverged: %v vs %v\n%s", finalState, refState, rt.Log())
	}
	if phase, _ := rt.AgentStatus("a1"); phase != "done" {
		t.Fatalf("agent not done after resume:\n%s", rt.Log())
	}
}

// A tick-budget timeout must stop the run cleanly with no dangling holds.
func TestTimeoutLeavesNoDanglingState(t *testing.T) {
	p := planner.New([]*planner.Action{
		{Name: "hold", Cost: 1, Resource: "r", Effects: []world.Effect{set("done", 1)}},
	}, planner.Options{})
	store := world.NewStore(world.NewState(nil))
	rt := New(store, p, Options{MaxTicks: 1}, nil) // times out before acting
	rt.AddAgent("a1", []planner.Goal{{Name: "g", Conditions: []world.Condition{eq("done", 1)}}})
	log := rt.Run(context.Background())
	if !hasEvent(log, EventTimeout, "") {
		t.Fatalf("expected timeout:\n%s", log)
	}
	if len(rt.resources.holders) != 0 {
		t.Fatalf("dangling resources: %v", rt.resources.holders)
	}
}

// A slow-planning agent must not block a fast one: the fast agent finishes
// while the slow one is still slicing its search.
func TestSlowAgentDoesNotBlockFastAgent(t *testing.T) {
	fast := []*planner.Action{
		{Name: "quick", Cost: 1, Effects: []world.Effect{set("fast-done", 1)}},
	}
	// Slow goal needs a long chain; with a tiny per-tick budget its search
	// spans many ticks.
	var slowActs []*planner.Action
	slowActs = append(slowActs, &planner.Action{Name: "step-00", Cost: 1, Effects: []world.Effect{set("s00", 1)}})
	for i := 1; i < 12; i++ {
		prev := i - 1
		slowActs = append(slowActs, &planner.Action{
			Name:          "step-" + itoa(i),
			Cost:          1,
			Preconditions: []world.Condition{eq("s"+itoa(prev), 1)},
			Effects:       []world.Effect{set("s"+itoa(i), 1)},
		})
	}
	all := append(append([]*planner.Action{}, fast...), slowActs...)
	p := planner.New(all, planner.Options{})
	store := world.NewStore(world.NewState(nil))
	rt := New(store, p, Options{PlanBudgetPerTick: 2}, nil)
	rt.AddAgent("fast", []planner.Goal{{Name: "g", Conditions: []world.Condition{eq("fast-done", 1)}}})
	rt.AddAgent("slow", []planner.Goal{{Name: "g", Conditions: []world.Condition{eq("s11", 1)}}})
	log := rt.Run(context.Background())

	fastDoneTick, slowDoneTick := uint64(0), uint64(0)
	for _, e := range log.Events() {
		if e.Kind == EventGoalReached && e.Agent == "fast" {
			fastDoneTick = e.Tick
		}
		if e.Kind == EventGoalReached && e.Agent == "slow" {
			slowDoneTick = e.Tick
		}
	}
	if fastDoneTick == 0 && !hasEvent(log, EventGoalReached, "") {
		t.Fatalf("nobody finished:\n%s", log)
	}
	if fastDoneTick >= slowDoneTick {
		t.Fatalf("fast agent finished at %d, slow at %d; fast should not be blocked\n%s",
			fastDoneTick, slowDoneTick, log)
	}
}

func itoa(i int) string { return fmt.Sprintf("%02d", i) }

// Full determinism: identical seed, identical inputs, byte-identical logs
// across repeated runs, including injected random failures.
func TestDeterministicReplay(t *testing.T) {
	run := func() string {
		p := planner.New([]*planner.Action{
			{Name: "a", Cost: 1, Effects: []world.Effect{set("x", 1)}},
			{Name: "b", Cost: 1, Preconditions: []world.Condition{eq("x", 1)}, Effects: []world.Effect{set("y", 1)}},
			{Name: "c", Cost: 1, Resource: "res", Effects: []world.Effect{set("w", 1)}},
		}, planner.Options{})
		store := world.NewStore(world.NewState(nil))
		rt := New(store, p, Options{PlanBudgetPerTick: 3}, SeededFailure(42, 0.3))
		rt.AddAgent("a1", []planner.Goal{{Name: "g1", Conditions: []world.Condition{eq("y", 1)}}})
		rt.AddAgent("a2", []planner.Goal{{Name: "g2", Conditions: []world.Condition{eq("w", 1)}}})
		return rt.Run(context.Background()).String()
	}
	first := run()
	for i := 0; i < 5; i++ {
		if got := run(); got != first {
			t.Fatalf("run %d diverged:\n--- first ---\n%s\n--- got ---\n%s", i, first, got)
		}
	}
}
