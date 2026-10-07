package runtime

import (
	"context"
	"math/rand"
	"sort"

	"github.com/cuixudong74/goal-action-planner-go/planner"
	"github.com/cuixudong74/goal-action-planner-go/world"
)

// Options tunes the scheduler. Zero values pick defaults.
type Options struct {
	// PlanBudgetPerTick caps search expansions per agent per tick, so a
	// slow planner cannot starve other agents.
	PlanBudgetPerTick int
	// MaxReplans bounds consecutive replans per agent before it fails.
	MaxReplans int
	// MaxResourceWaits bounds ticks an agent waits for a busy resource
	// before treating the action as failed and replanning.
	MaxResourceWaits int
	// MaxTicks bounds the whole run (timeout). Zero means no limit.
	MaxTicks uint64
}

func (o Options) withDefaults() Options {
	if o.PlanBudgetPerTick <= 0 {
		o.PlanBudgetPerTick = 256
	}
	if o.MaxReplans <= 0 {
		o.MaxReplans = 8
	}
	if o.MaxResourceWaits <= 0 {
		o.MaxResourceWaits = 16
	}
	return o
}

// FailFunc decides whether an action attempt fails. It is evaluated in
// deterministic (tick, agent-ID) order, so a pure function of its
// arguments — or a seeded RNG consumed in that order — yields
// reproducible runs.
type FailFunc func(tick uint64, agent, action string) bool

// Runtime drives a set of agents sharing one world store. The scheduler is
// a deterministic tick loop: every tick, agents act in ascending ID order,
// each performing at most one planning slice or one action. World mutation
// is serialized through the store, so results never depend on goroutine
// scheduling or map iteration order.
type Runtime struct {
	store     *world.Store
	planner   *planner.Planner
	resources *Resources
	log       *Log
	opts      Options
	failFn    FailFunc

	agents []*agent // sorted by ID
	tick   uint64
}

// New builds a Runtime. Agents are added with AddAgent before Run.
func New(store *world.Store, p *planner.Planner, opts Options, failFn FailFunc) *Runtime {
	return &Runtime{
		store:     store,
		planner:   p,
		resources: newResources(),
		log:       &Log{},
		opts:      opts.withDefaults(),
		failFn:    failFn,
	}
}

// AddAgent registers an agent with its goals. IDs must be unique.
func (r *Runtime) AddAgent(id string, goals []planner.Goal) {
	r.agents = append(r.agents, newAgent(id, goals))
}

// Log returns the event log accumulated so far.
func (r *Runtime) Log() *Log { return r.log }

// Tick returns the current logical clock.
func (r *Runtime) Tick() uint64 { return r.tick }

// AgentStatus reports an agent's phase and, when failed, the cause.
func (r *Runtime) AgentStatus(id string) (phase string, cause string) {
	for _, a := range r.agents {
		if a.id == id {
			switch a.phase {
			case phasePlanning:
				return "planning", ""
			case phaseExecuting:
				return "executing", ""
			case phaseDone:
				return "done", ""
			case phaseFailed:
				return "failed", a.failCause
			}
		}
	}
	return "unknown", ""
}

// Run drives the scheduler until all agents finish, the context is
// cancelled, or the tick budget is exhausted. It is resumable: after a
// cancellation, calling Run again continues from the exact interruption
// point with no partial actions and no dangling resource holds.
func (r *Runtime) Run(ctx context.Context) *Log {
	// Agents act in ascending ID order every tick: deterministic.
	sort.Slice(r.agents, func(i, j int) bool { return r.agents[i].id < r.agents[j].id })

	for {
		if err := ctx.Err(); err != nil {
			for _, a := range r.agents {
				a.cleanup(r)
			}
			r.log.append(Event{Tick: r.tick, Agent: "*", Kind: EventCancelled})
			return r.log
		}
		if r.opts.MaxTicks > 0 && r.tick >= r.opts.MaxTicks {
			for _, a := range r.agents {
				a.cleanup(r)
			}
			r.log.append(Event{Tick: r.tick, Agent: "*", Kind: EventTimeout})
			return r.log
		}
		if r.allSettled() {
			return r.log
		}
		r.step()
		r.tick++
	}
}

func (r *Runtime) allSettled() bool {
	for _, a := range r.agents {
		if a.phase != phaseDone && a.phase != phaseFailed {
			return false
		}
	}
	return true
}

// step advances every unsettled agent by one deterministic slice.
func (r *Runtime) step() {
	for _, a := range r.agents {
		switch a.phase {
		case phasePlanning:
			if !a.planning {
				a.beginPlanning(r)
			}
			a.stepPlanning(r)
		case phaseExecuting:
			a.stepExecuting(r)
		}
	}
}

// SeededFailure returns a FailFunc that fails each action attempt with
// probability p, driven by a seeded RNG. Because the scheduler evaluates
// it in fixed (tick, agent-ID) order, identical seeds and inputs reproduce
// identical failure sequences.
func SeededFailure(seed int64, p float64) FailFunc {
	rng := rand.New(rand.NewSource(seed))
	return func(tick uint64, agent, action string) bool {
		return rng.Float64() < p
	}
}

// ScriptFailure fails the named action for the named agent exactly n times
// (counted in deterministic evaluation order), then lets it succeed.
// Useful for deviation-and-replan tests.
func ScriptFailure(failures map[string]int) FailFunc {
	remaining := map[string]int{}
	for k, v := range failures {
		remaining[k] = v
	}
	return func(tick uint64, agent, action string) bool {
		key := agent + "/" + action
		if remaining[key] > 0 {
			remaining[key]--
			return true
		}
		return false
	}
}
