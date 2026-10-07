package goap

import (
	"context"
	"fmt"
	"sort"
	"sync"
	"time"
)

// RuntimeConfig tunes the tick loop.
type RuntimeConfig struct {
	// TurnTimeout bounds a single agent turn. 0 means no timeout.
	// A turn that exceeds its budget is aborted between atomic steps —
	// never mid-action.
	TurnTimeout time.Duration
	// ResourceLeaseTicks is how many ticks (counting the acquisition
	// tick) a resource hold remains valid. 0 means 1: the hold lasts
	// to the end of the current tick, so agents acting later in the
	// same tick contend for it, and it is free again next tick. Holds
	// always expire — an interrupted agent can never leave a dangling
	// lock behind.
	ResourceLeaseTicks int
	// Executor configures re-planning inside turns.
	Executor ExecutorConfig
}

// Runtime drives multiple agents that share one World.
//
// Determinism model: the runtime advances in ticks. Within a tick,
// agents take their turn sequentially in lexicographic ID order, so
// the outcome never depends on goroutine scheduling or map iteration
// order. Resource contention is resolved by that same turn order and
// every decision is recorded in the shared event log, which makes the
// winner/loser of each contention explainable. A slow agent is bounded
// by TurnTimeout and cannot block the others beyond it.
type Runtime struct {
	world   *World
	planner *Planner
	seed    uint64
	cfg     RuntimeConfig

	mu        sync.Mutex
	tick      uint64
	resources map[string]resourceHold // resource name -> hold
	events    []Event
	agents    []Agent // kept sorted by ID
}

// resourceHold records who holds a resource and through which tick.
type resourceHold struct {
	holder string
	expiry uint64 // hold is valid for ticks <= expiry
}

// Agent participates in the runtime. Implementations must be
// deterministic given the same snapshot, tick and Handle random source.
type Agent interface {
	ID() string
	// Act is invoked once per tick with a consistent world snapshot.
	Act(ctx context.Context, h *Handle, snap State, tick uint64)
}

// AgentFunc adapts a function to Agent.
type AgentFunc struct {
	Name string
	Fn   func(ctx context.Context, h *Handle, snap State, tick uint64)
}

// ID implements Agent.
func (a AgentFunc) ID() string { return a.Name }

// Act implements Agent.
func (a AgentFunc) Act(ctx context.Context, h *Handle, snap State, tick uint64) {
	a.Fn(ctx, h, snap, tick)
}

// NewRuntime creates a runtime over w. The seed makes any runtime-level
// pseudo-random choices reproducible.
func NewRuntime(w *World, p *Planner, seed uint64, cfg RuntimeConfig) *Runtime {
	return &Runtime{
		world:     w,
		planner:   p,
		seed:      seed,
		cfg:       cfg,
		resources: map[string]resourceHold{},
	}
}

// World returns the shared world.
func (r *Runtime) World() *World { return r.world }

// AddAgent registers an agent. Registration order does not matter;
// turns always run in lexicographic ID order.
func (r *Runtime) AddAgent(a Agent) {
	r.mu.Lock()
	defer r.mu.Unlock()
	r.agents = append(r.agents, a)
	sort.Slice(r.agents, func(i, j int) bool { return r.agents[i].ID() < r.agents[j].ID() })
}

// Events returns a copy of the shared, deterministically ordered log.
func (r *Runtime) Events() []Event {
	r.mu.Lock()
	defer r.mu.Unlock()
	out := make([]Event, len(r.events))
	copy(out, r.events)
	return out
}

func (r *Runtime) emit(kind EventKind, agent, action, detail string) {
	r.mu.Lock()
	defer r.mu.Unlock()
	r.events = append(r.events, Event{
		Seq:     len(r.events),
		Agent:   agent,
		Kind:    kind,
		Action:  action,
		Detail:  detail,
		Version: r.world.Version(),
	})
}

// Run advances the runtime for the given number of ticks or until the
// context is done. It returns the shared event log.
func (r *Runtime) Run(ctx context.Context, ticks int) []Event {
	for t := 0; t < ticks; t++ {
		if ctx.Err() != nil {
			break
		}
		r.mu.Lock()
		r.tick++
		tick := r.tick
		// Purge expired holds: deterministic, and guarantees that an
		// interrupted agent never blocks a resource forever.
		for res, h := range r.resources {
			if h.expiry < tick {
				delete(r.resources, res)
			}
		}
		agents := make([]Agent, len(r.agents))
		copy(agents, r.agents)
		r.mu.Unlock()

		for _, a := range agents {
			if ctx.Err() != nil {
				break
			}
			r.runTurn(ctx, a, tick)
		}
	}
	return r.Events()
}

// runTurn executes one agent's turn with its timeout budget and
// guarantees no resource holds leak past the turn.
func (r *Runtime) runTurn(ctx context.Context, a Agent, tick uint64) {
	turnCtx := ctx
	cancel := func() {}
	if r.cfg.TurnTimeout > 0 {
		turnCtx, cancel = context.WithTimeout(ctx, r.cfg.TurnTimeout)
	}
	defer cancel()

	snap, _ := r.world.Snapshot()
	h := &Handle{rt: r, agent: a.ID(), tick: tick}
	a.Act(turnCtx, h, snap, tick)
	if turnCtx.Err() != nil && ctx.Err() == nil {
		r.emit(EventAborted, a.ID(), "", "turn timeout")
	}
}

// acquire grants a resource to an agent if it is free (or its lease
// expired) or already held by that agent. The decision and its reason
// are logged, so contention outcomes are explainable.
func (r *Runtime) acquire(agent, resource string) bool {
	lease := r.cfg.ResourceLeaseTicks
	if lease <= 0 {
		lease = 1
	}
	r.mu.Lock()
	tick := r.tick
	hold, taken := r.resources[resource]
	if !taken || hold.expiry < tick || hold.holder == agent {
		r.resources[resource] = resourceHold{holder: agent, expiry: tick + uint64(lease) - 1}
		r.mu.Unlock()
		return true
	}
	holder := hold.holder
	r.mu.Unlock()
	r.emit(EventResourceWait, agent, "",
		fmt.Sprintf("resource %q held by %q until tick %d", resource, holder, hold.expiry))
	return false
}

// release drops one resource hold; it is a no-op if the agent does
// not hold it.
func (r *Runtime) release(agent, resource string) {
	r.mu.Lock()
	defer r.mu.Unlock()
	if h, ok := r.resources[resource]; ok && h.holder == agent {
		delete(r.resources, resource)
	}
}

// Handle gives an agent deterministic access to the shared runtime
// during its turn.
type Handle struct {
	rt    *Runtime
	agent string
	tick  uint64
	rand  uint64 // lazily initialised deterministic PRNG state
}

// Snapshot returns a consistent view of the shared world.
func (h *Handle) Snapshot() (State, uint64) { return h.rt.world.Snapshot() }

// Planner returns the shared planner (planning itself is pure and
// snapshot-based, so agents cannot disturb each other while planning).
func (h *Handle) Planner() *Planner { return h.rt.planner }

// Acquire attempts to take a scarce resource for the rest of the turn.
// The winner of a contention is determined by tick + turn order and
// the decision is logged with the current holder's identity.
func (h *Handle) Acquire(resource string) bool { return h.rt.acquire(h.agent, resource) }

// Release drops a resource hold early.
func (h *Handle) Release(resource string) { h.rt.release(h.agent, resource) }

// Rand returns a deterministic pseudo-random uint64 derived from the
// runtime seed, the agent ID and the tick. Identical seeds and input
// sequences produce identical streams regardless of scheduling.
func (h *Handle) Rand() uint64 {
	if h.rand == 0 {
		h.rand = h.rt.seed ^ (h.tick * 0x9E3779B97F4A7C15)
		for _, c := range h.agent {
			h.rand = (h.rand ^ uint64(c)) * 0x100000001B3
		}
		if h.rand == 0 {
			h.rand = 0x2545F4914F6CDD1D
		}
	}
	// splitmix64 step
	h.rand += 0x9E3779B97F4A7C15
	z := h.rand
	z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9
	z = (z ^ (z >> 27)) * 0x94D049BB133111EB
	return z ^ (z >> 31)
}

// RunGoal plans (or reuses plan) and executes towards the goal within
// this turn. Actions that declare Resources are only applied while the
// agent holds them; the handle auto-attempts acquisition and a denial
// surfaces as an action failure with the holder's identity, triggering
// the executor's normal re-plan path. All events flow into the shared
// runtime log with deterministic sequence numbers.
func (h *Handle) RunGoal(ctx context.Context, goal []Condition, plan *Plan) RunResult {
	runner := RunnerFunc(func(ctx context.Context, w *World, a *Action, base uint64) (uint64, error) {
		for _, res := range a.Resources {
			if !h.rt.acquire(h.agent, res) {
				h.rt.mu.Lock()
				holder := h.rt.resources[res].holder
				h.rt.mu.Unlock()
				return base, fmt.Errorf("resource %q held by %q", res, holder)
			}
		}
		return w.ApplyIfVersion(a.Effects, base)
	})
	ex := NewExecutor(h.rt.planner, runner, h.rt.cfg.Executor)
	res := ex.Run(ctx, h.agent, h.rt.world, goal, plan)
	for _, ev := range res.Events {
		h.rt.emit(ev.Kind, ev.Agent, ev.Action, ev.Detail)
	}
	return res
}
