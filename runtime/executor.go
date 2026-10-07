package runtime

import (
	"fmt"

	"github.com/cuixudong74/goal-action-planner-go/planner"
	"github.com/cuixudong74/goal-action-planner-go/world"
)

// agentPhase is the executor state machine stage.
type agentPhase int

const (
	phasePlanning agentPhase = iota
	phaseExecuting
	phaseDone
	phaseFailed
)

// agent is one independent planning+execution unit. Agents share the world
// store but never share plans, searches or resources, so they cannot
// cross-interfere except through the world itself.
type agent struct {
	id    string
	goals []planner.Goal

	phase agentPhase

	// planning state
	prep       planner.Prepared
	search     *planner.Search
	planFrom   uint64 // store version the in-progress search is based on
	early      *planner.Result
	replans    int
	lastStatus planner.Status

	// execution state
	plan       planner.Plan
	next       int // index of the next action to execute
	resWaits   int // consecutive ticks denied a resource
	heldRes    string
	inProgress bool // a multi-tick action is underway
	progress   int  // ticks elapsed on the in-progress action
	planning   bool // a search object is live
	failCause  string
}

func newAgent(id string, goals []planner.Goal) *agent {
	return &agent{id: id, goals: goals}
}

// beginPlanning starts (or restarts) goal preparation and search from the
// current store snapshot. The version is remembered so the finished plan
// can be validated against later world drift.
func (a *agent) beginPlanning(rt *Runtime) {
	state, version := rt.store.Snapshot()
	prep, early := rt.planner.Prepare(a.goals)
	a.prep = prep
	a.early = early
	a.planFrom = version
	a.planning = true
	a.search = nil
	if early == nil {
		a.search = rt.planner.NewSearch(state, prep)
	}
	rt.log.append(Event{Tick: rt.tick, Agent: a.id, Kind: EventPlanStarted,
		Detail: fmt.Sprintf("goals=%v version=%d", goalNamesOf(prep), version)})
}

func goalNamesOf(p planner.Prepared) []string {
	n := make([]string, len(p.Goals))
	for i, g := range p.Goals {
		n[i] = g.Name
	}
	return n
}

// stepPlanning advances the search by the per-tick budget. A slow search
// only consumes its own slice, never other agents' ticks.
func (a *agent) stepPlanning(rt *Runtime) {
	if a.early != nil {
		a.finishPlanning(rt, *a.early)
		return
	}
	if !a.search.Step(rt.opts.PlanBudgetPerTick) {
		return // still searching; yields the rest of the tick to others
	}
	res := a.search.Result()
	// The world may have moved on while we were planning across ticks.
	if _, version := rt.store.Snapshot(); version != a.planFrom {
		rt.log.append(Event{Tick: rt.tick, Agent: a.id, Kind: EventPlanInvalidated,
			Detail: fmt.Sprintf("planned-on=%d now=%d", a.planFrom, version)})
		a.beginPlanning(rt)
		return
	}
	a.finishPlanning(rt, rt.planner.Finish(snapshotOf(rt, a.planFrom), a.planFrom, a.prep, res))
}

func snapshotOf(rt *Runtime, _ uint64) world.State {
	st, _ := rt.store.Snapshot()
	return st
}

func (a *agent) finishPlanning(rt *Runtime, res planner.Result) {
	a.planning = false
	a.lastStatus = res.Status
	if res.Status != planner.StatusOK {
		rt.log.append(Event{Tick: rt.tick, Agent: a.id, Kind: EventAgentFailed,
			Detail: fmt.Sprintf("status=%s unreach=%v conflicts=%d", res.Status, res.UnreachableGoals, len(res.Conflicts))})
		a.phase = phaseFailed
		a.failCause = res.Status.String()
		return
	}
	a.plan = res.Plan
	a.next = 0
	rt.log.append(Event{Tick: rt.tick, Agent: a.id, Kind: EventPlanned,
		Detail: fmt.Sprintf("actions=%v cost=%g", a.plan.Names(), a.plan.TotalCost)})
	if len(a.plan.Actions) == 0 {
		a.phase = phaseDone // goals already satisfied
		rt.log.append(Event{Tick: rt.tick, Agent: a.id, Kind: EventGoalReached})
		return
	}
	a.phase = phaseExecuting
}

// stepExecuting executes exactly one action (or one failure/replan cycle)
// per tick, keeping the interleaving with other agents deterministic.
func (a *agent) stepExecuting(rt *Runtime) {
	if a.next >= len(a.plan.Actions) {
		a.phase = phaseDone
		rt.log.append(Event{Tick: rt.tick, Agent: a.id, Kind: EventGoalReached})
		return
	}
	act := a.plan.Actions[a.next]

	// Resource arbitration: deterministic because agents act in ID order.
	if act.Resource != "" && a.heldRes != act.Resource {
		if rt.resources.acquire(act.Resource, a.id) {
			a.heldRes = act.Resource
			rt.log.append(Event{Tick: rt.tick, Agent: a.id, Kind: EventResourceAcquired, Action: act.Name,
				Detail: act.Resource})
		} else {
			rt.log.append(Event{Tick: rt.tick, Agent: a.id, Kind: EventResourceDenied, Action: act.Name,
				Detail: act.Resource})
			a.resWaits++
			if a.resWaits > rt.opts.MaxResourceWaits {
				a.replan(rt, "resource-starved:"+act.Resource)
			}
			return
		}
	}
	a.resWaits = 0

	if !a.inProgress {
		rt.log.append(Event{Tick: rt.tick, Agent: a.id, Kind: EventActionStart, Action: act.Name})

		// Injected failure: the attempt fails before anything is applied.
		// Atomicity means there is no partial action to clean up.
		if rt.failFn != nil && rt.failFn(rt.tick, a.id, act.Name) {
			rt.log.append(Event{Tick: rt.tick, Agent: a.id, Kind: EventActionFailed, Action: act.Name})
			a.releaseHeld(rt)
			a.replan(rt, "action-failed:"+act.Name)
			return
		}

		// Preconditions must hold for the action to start at all.
		if state, _ := rt.store.Snapshot(); !state.Satisfies(act.Preconditions) {
			rt.log.append(Event{Tick: rt.tick, Agent: a.id, Kind: EventPrecondBroken, Action: act.Name})
			a.releaseHeld(rt)
			a.replan(rt, "precond-broken:"+act.Name)
			return
		}
		a.inProgress = true
		a.progress = 0
	}

	// Long actions occupy their resource across ticks; effects land
	// atomically only at completion.
	dur := act.Duration
	if dur < 1 {
		dur = 1
	}
	a.progress++
	if a.progress < dur {
		return
	}
	a.inProgress = false

	// Revalidate preconditions against the live world before applying:
	// the world may have moved on while the action was running.
	state, version := rt.store.Snapshot()
	if !state.Satisfies(act.Preconditions) {
		rt.log.append(Event{Tick: rt.tick, Agent: a.id, Kind: EventPrecondBroken, Action: act.Name})
		a.releaseHeld(rt)
		a.replan(rt, "precond-broken:"+act.Name)
		return
	}

	// Apply atomically; a version race means the world changed under us.
	res := rt.store.ApplyIf(version, act.Preconditions, act.Effects)
	if !res.OK {
		rt.log.append(Event{Tick: rt.tick, Agent: a.id, Kind: EventPrecondBroken, Action: act.Name,
			Detail: res.FailedCond})
		a.releaseHeld(rt)
		a.replan(rt, "apply-conflict:"+act.Name)
		return
	}

	// Verify postconditions: effects must have materialized as expected.
	if len(act.Postconditions) > 0 {
		if st, _ := rt.store.Snapshot(); !st.Satisfies(act.Postconditions) {
			rt.log.append(Event{Tick: rt.tick, Agent: a.id, Kind: EventEffectMismatch, Action: act.Name})
			a.releaseHeld(rt)
			a.replan(rt, "effect-mismatch:"+act.Name)
			return
		}
	}

	rt.log.append(Event{Tick: rt.tick, Agent: a.id, Kind: EventActionApplied, Action: act.Name})
	a.releaseHeld(rt)
	a.next++
}

// replan discards the rest of the current plan and re-plans from the live
// world state. Actions already applied stay applied (their effects are in
// the world, so the new plan never redoes them); preconditions still
// required are re-derived by the planner, never skipped.
func (a *agent) replan(rt *Runtime, cause string) {
	a.inProgress = false // abort any multi-tick action: no partial effects
	a.replans++
	rt.log.append(Event{Tick: rt.tick, Agent: a.id, Kind: EventReplan, Detail: cause})
	if a.replans > rt.opts.MaxReplans {
		a.phase = phaseFailed
		a.failCause = "replan-budget-exhausted:" + cause
		rt.log.append(Event{Tick: rt.tick, Agent: a.id, Kind: EventAgentFailed, Detail: a.failCause})
		return
	}
	a.phase = phasePlanning
	a.beginPlanning(rt)
}

func (a *agent) releaseHeld(rt *Runtime) {
	if a.heldRes != "" {
		rt.resources.release(a.heldRes, a.id)
		rt.log.append(Event{Tick: rt.tick, Agent: a.id, Kind: EventResourceReleased, Detail: a.heldRes})
		a.heldRes = ""
	}
}

// cleanup releases any dangling resource hold; used on cancel/timeout.
func (a *agent) cleanup(rt *Runtime) {
	a.inProgress = false
	a.releaseHeld(rt)
	for _, res := range rt.resources.releaseAll(a.id) {
		rt.log.append(Event{Tick: rt.tick, Agent: a.id, Kind: EventResourceReleased, Detail: res})
	}
}
