package goap

import (
	"context"
	"fmt"
)

// EventKind classifies entries in the execution log.
type EventKind int

const (
	EventPlanFound EventKind = iota
	EventActionStarted
	EventActionApplied
	EventActionFailed
	EventStaleState // snapshot mismatch detected before a step
	EventReplan     // a re-plan was triggered mid-execution
	EventGoalAchieved
	EventAborted      // execution cancelled or timed out
	EventResourceWait // had to wait for / was denied a resource
)

func (k EventKind) String() string {
	switch k {
	case EventPlanFound:
		return "plan_found"
	case EventActionStarted:
		return "action_started"
	case EventActionApplied:
		return "action_applied"
	case EventActionFailed:
		return "action_failed"
	case EventStaleState:
		return "stale_state"
	case EventReplan:
		return "replan"
	case EventGoalAchieved:
		return "goal_achieved"
	case EventAborted:
		return "aborted"
	case EventResourceWait:
		return "resource_wait"
	}
	return "unknown"
}

// Event is one deterministic, ordered entry in the execution log.
type Event struct {
	Seq     int
	Agent   string
	Kind    EventKind
	Action  string
	Detail  string
	Version uint64
}

// Outcome is the final, distinguishable result of a run.
type Outcome int

const (
	OutcomeSuccess Outcome = iota
	OutcomeNoPlan
	OutcomeFailed   // an action failed and re-planning could not recover
	OutcomeCanceled // context canceled or deadline exceeded
)

func (o Outcome) String() string {
	switch o {
	case OutcomeSuccess:
		return "success"
	case OutcomeNoPlan:
		return "no_plan"
	case OutcomeFailed:
		return "failed"
	case OutcomeCanceled:
		return "canceled"
	}
	return "unknown"
}

// Runner applies one action to the world. The default runner applies
// the action's declared effects atomically; tests can inject failures
// or environmental drift.
type Runner interface {
	// Run executes the action. baseVersion is the world version the
	// executor validated preconditions against; implementations should
	// use ApplyIfVersion so concurrent drift is detected. It returns
	// the new world version or an error.
	Run(ctx context.Context, w *World, a *Action, baseVersion uint64) (uint64, error)
}

// RunnerFunc adapts a function to Runner.
type RunnerFunc func(ctx context.Context, w *World, a *Action, baseVersion uint64) (uint64, error)

// Run implements Runner.
func (f RunnerFunc) Run(ctx context.Context, w *World, a *Action, baseVersion uint64) (uint64, error) {
	return f(ctx, w, a, baseVersion)
}

// DefaultRunner applies declared effects with a version check. The
// effects land atomically: an interrupted or conflicting run changes
// nothing, so no action is ever left half-applied.
func DefaultRunner() Runner {
	return RunnerFunc(func(ctx context.Context, w *World, a *Action, baseVersion uint64) (uint64, error) {
		return w.ApplyIfVersion(a.Effects, baseVersion)
	})
}

// ExecutorConfig controls re-planning behaviour.
type ExecutorConfig struct {
	MaxReplans int // 0 => default of 8
}

// Executor executes plans against a World, re-planning from the
// interruption point when the world drifts or actions fail.
type Executor struct {
	planner *Planner
	runner  Runner
	cfg     ExecutorConfig
}

// NewExecutor builds an executor; a nil runner selects DefaultRunner.
func NewExecutor(p *Planner, r Runner, cfg ExecutorConfig) *Executor {
	if r == nil {
		r = DefaultRunner()
	}
	if cfg.MaxReplans <= 0 {
		cfg.MaxReplans = 8
	}
	return &Executor{planner: p, runner: r, cfg: cfg}
}

// RunResult is the deterministic record of an execution.
type RunResult struct {
	Outcome Outcome
	Events  []Event
	// Executed lists the actions whose effects were actually applied,
	// in order. Already-effective actions are never re-executed.
	Executed []string
}

// Run plans (if plan is nil) and executes towards the goal. Before
// every step the action's preconditions are validated against a fresh
// world snapshot; a mismatch or an action failure triggers a re-plan
// from the current state — never a blind retry of the stale plan.
//
// Properties:
//   - already-effective actions are skipped, never re-executed;
//   - preconditions are never skipped: if they no longer hold, the
//     executor re-plans instead of proceeding;
//   - context cancellation stops between atomic steps and is reported
//     as OutcomeCanceled; effects already applied stay applied, so a
//     later resume re-plans from the same world and converges to the
//     same end state as an uninterrupted run.
func (e *Executor) Run(ctx context.Context, agent string, w *World, goal []Condition, plan *Plan) RunResult {
	res := RunResult{}
	seq := 0
	emit := func(kind EventKind, action, detail string, version uint64) {
		res.Events = append(res.Events, Event{Seq: seq, Agent: agent, Kind: kind, Action: action, Detail: detail, Version: version})
		seq++
	}

	replans := 0
	for {
		if err := ctx.Err(); err != nil {
			emit(EventAborted, "", "context: "+err.Error(), w.Version())
			res.Outcome = OutcomeCanceled
			return res
		}
		snap, version := w.Snapshot()
		if snap.Satisfies(goal) {
			emit(EventGoalAchieved, "", "", version)
			res.Outcome = OutcomeSuccess
			return res
		}
		if plan == nil {
			p, err := e.planner.Plan(snap, goal)
			if err != nil {
				emit(EventAborted, "", "planning: "+err.Error(), version)
				res.Outcome = OutcomeNoPlan
				return res
			}
			plan = p
			emit(EventPlanFound, "", fmt.Sprintf("cost=%g steps=%v", p.Cost, p.Names()), version)
		}

		step := 0
		replan := false
		for step < plan.Len() {
			a := plan.Actions[step]
			if err := ctx.Err(); err != nil {
				emit(EventAborted, a.Name, "context: "+err.Error(), w.Version())
				res.Outcome = OutcomeCanceled
				return res
			}
			snap, version := w.Snapshot()
			if snap.Satisfies(goal) {
				break // remaining steps are unnecessary
			}
			if !a.Enabled(snap) {
				emit(EventStaleState, a.Name, "preconditions no longer hold in "+snap.String(), version)
				replan = true
				break
			}
			if len(snap.Missing(a.EffectConditions())) == 0 {
				step++ // effects already present: skip, do not re-execute
				continue
			}
			emit(EventActionStarted, a.Name, "", version)
			newVersion, err := e.runner.Run(ctx, w, a, version)
			if err != nil {
				emit(EventActionFailed, a.Name, err.Error(), w.Version())
				replan = true
				break
			}
			// Verify the effects actually landed as declared.
			post, postVersion := w.Snapshot()
			if missing := post.Missing(a.EffectConditions()); len(missing) > 0 {
				emit(EventActionFailed, a.Name, fmt.Sprintf("effects missing after run: %v", missing), postVersion)
				replan = true
				break
			}
			emit(EventActionApplied, a.Name, "", newVersion)
			res.Executed = append(res.Executed, a.Name)
			step++
		}
		if !replan && step >= plan.Len() {
			plan = nil // finished plan; loop re-checks the goal
			continue
		}
		if replan {
			replans++
			if replans > e.cfg.MaxReplans {
				emit(EventAborted, "", "replan budget exhausted", w.Version())
				res.Outcome = OutcomeFailed
				return res
			}
			emit(EventReplan, "", fmt.Sprintf("attempt %d", replans), w.Version())
			plan = nil
		}
	}
}
