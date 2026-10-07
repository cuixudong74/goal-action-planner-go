package goap

import (
	"container/heap"
	"errors"
	"fmt"
	"strings"
)

// Failure reasons returned by the planner. They are distinguishable so
// callers can tell "no plan exists" apart from "search was cut off".
var (
	// ErrNoPlan means the search space was exhausted without reaching
	// the goal: the goal is unreachable with the available actions.
	ErrNoPlan = errors.New("goap: no plan found")
	// ErrSearchLimit means the search hit a configured budget
	// (iterations or depth) before exhausting the space.
	ErrSearchLimit = errors.New("goap: search limit reached")
)

// Plan is an ordered action sequence plus its total cost.
type Plan struct {
	Actions []*Action
	Cost    float64
}

// Len returns the number of steps.
func (p *Plan) Len() int { return len(p.Actions) }

// Names returns the action names in order (handy for assertions).
func (p *Plan) Names() []string {
	out := make([]string, len(p.Actions))
	for i, a := range p.Actions {
		out[i] = a.Name
	}
	return out
}

// PlannerConfig bounds the search so it always terminates.
type PlannerConfig struct {
	MaxIterations int // max number of state expansions; 0 => default
	MaxDepth      int // max plan length; 0 => default
}

// Planner finds deterministic action sequences for goals.
type Planner struct {
	actions *ActionSet
	cfg     PlannerConfig
}

// NewPlanner builds a planner over the given action set.
func NewPlanner(actions *ActionSet, cfg PlannerConfig) *Planner {
	return &Planner{actions: actions, cfg: cfg.withDefaults()}
}

func (c PlannerConfig) withDefaults() PlannerConfig {
	if c.MaxIterations <= 0 {
		c.MaxIterations = 10000
	}
	if c.MaxDepth <= 0 {
		c.MaxDepth = 64
	}
	return c
}

// stateKey renders a state canonically so map lookups and visited-set
// membership are deterministic.
func stateKey(s State) string {
	var b strings.Builder
	for _, k := range s.SortedKeys() {
		fmt.Fprintf(&b, "%s\x00%v\x01", k, s[k])
	}
	return b.String()
}

// node is one search-tree entry.
type node struct {
	state  State
	g      float64
	path   []*Action
	groups map[string]bool // mutex groups already used on this path
	seq    int             // insertion order, final deterministic tiebreak
}

// comparePaths orders two action sequences lexicographically by action
// name; a strict prefix is smaller than its extension.
func comparePaths(a, b []*Action) int {
	for i := 0; i < len(a) && i < len(b); i++ {
		if a[i].Name != b[i].Name {
			if a[i].Name < b[i].Name {
				return -1
			}
			return 1
		}
	}
	switch {
	case len(a) < len(b):
		return -1
	case len(a) > len(b):
		return 1
	}
	return 0
}

// nodeHeap is a priority queue ordered by (g, path-lex, seq). Because
// the heuristic is zero this is uniform-cost search: the first time a
// goal state is popped it is optimal, and equal-cost plans are popped
// in lexicographic path order — that is the tie-breaking rule.
type nodeHeap []*node

func (h nodeHeap) Len() int { return len(h) }
func (h nodeHeap) Less(i, j int) bool {
	a, b := h[i], h[j]
	if a.g != b.g {
		return a.g < b.g
	}
	if c := comparePaths(a.path, b.path); c != 0 {
		return c < 0
	}
	return a.seq < b.seq
}
func (h nodeHeap) Swap(i, j int) { h[i], h[j] = h[j], h[i] }
func (h *nodeHeap) Push(x any)   { *h = append(*h, x.(*node)) }
func (h *nodeHeap) Pop() (x any) { old := *h; x = old[len(old)-1]; *h = old[:len(old)-1]; return x }

// Plan searches for the cheapest action sequence taking state to a
// state satisfying the goal conditions.
//
// Determinism guarantees:
//   - actions are expanded in lexicographic name order;
//   - the queue pops by (total cost, lexicographic action-name sequence,
//     insertion order), so among equivalent cheapest plans the one whose
//     action names sort first wins;
//   - the search always terminates: it is bounded by MaxIterations and
//     MaxDepth, and each reachable state is expanded at most once.
func (p *Planner) Plan(initial State, goal []Condition) (*Plan, error) {
	if initial.Satisfies(goal) {
		return &Plan{}, nil
	}
	pq := &nodeHeap{}
	heap.Init(pq)
	seq := 0
	heap.Push(pq, &node{state: initial.Clone(), groups: map[string]bool{}, seq: seq})
	seq++
	best := map[string]float64{stateKey(initial): 0}
	expanded := 0
	limitHit := false

	for pq.Len() > 0 {
		n := heap.Pop(pq).(*node)
		key := stateKey(n.state)
		if g, ok := best[key]; ok && n.g > g {
			continue // stale queue entry
		}
		if n.state.Satisfies(goal) {
			return &Plan{Actions: n.path, Cost: n.g}, nil
		}
		if expanded >= p.cfg.MaxIterations {
			limitHit = true
			break
		}
		expanded++
		if len(n.path) >= p.cfg.MaxDepth {
			continue
		}
		for _, a := range p.actions.All() { // deterministic order
			if !a.Enabled(n.state) || conflicts(a, n.groups) {
				continue
			}
			next := n.state.Apply(a.Effects)
			ng := n.g + a.Cost
			nk := stateKey(next)
			if g, ok := best[nk]; ok && ng >= g {
				continue
			}
			best[nk] = ng
			groups := make(map[string]bool, len(n.groups)+len(a.Mutex))
			for gname := range n.groups {
				groups[gname] = true
			}
			for _, gname := range a.Mutex {
				groups[gname] = true
			}
			path := make([]*Action, len(n.path)+1)
			copy(path, n.path)
			path[len(n.path)] = a
			heap.Push(pq, &node{state: next, g: ng, path: path, groups: groups, seq: seq})
			seq++
		}
	}
	if limitHit {
		return nil, ErrSearchLimit
	}
	return nil, ErrNoPlan
}

// conflicts reports whether action a uses a mutex group already present
// on the current path.
func conflicts(a *Action, groups map[string]bool) bool {
	for _, g := range a.Mutex {
		if groups[g] {
			return true
		}
	}
	return false
}

// PlanGoals evaluates multiple goals in deterministic priority order
// (higher priority first, ties by name) and merges the feasible ones:
// each planned goal's actions extend the state produced by the goals
// committed before it, so the concatenation of the returned plans is
// one consistent combined plan.
//
// Conclusions are distinguishable per goal: satisfied, planned,
// unreachable, conflicting (contradicts an already-committed
// higher-or-equal priority goal) or deferred (dependency not met).
func (p *Planner) PlanGoals(initial State, goals []*Goal) []GoalResult {
	ordered := orderGoals(goals)
	results := make([]GoalResult, 0, len(ordered))
	committed := initial.Clone()
	committedDesired := map[string]Value{} // facts committed goals require
	statusByName := map[string]GoalStatus{}

	for _, g := range ordered {
		res := GoalResult{Goal: g}
		// Dependencies must be satisfied or planned-for already.
		depsOK := true
		for _, dep := range g.DependsOn {
			st, ok := statusByName[dep]
			if !ok || (st != GoalSatisfied && st != GoalPlanned) {
				depsOK = false
				break
			}
		}
		switch {
		case !depsOK:
			res.Status = GoalDeferred
		case committed.Satisfies(g.Desired):
			res.Status = GoalSatisfied
		case contradicts(g.Desired, committedDesired):
			res.Status = GoalConflicting
		default:
			plan, err := p.Plan(committed, g.Desired)
			if err != nil {
				res.Status = GoalUnreachable
			} else {
				res.Status = GoalPlanned
				res.Plan = plan
				for _, a := range plan.Actions {
					committed = committed.Apply(a.Effects)
				}
			}
		}
		if res.Status == GoalSatisfied || res.Status == GoalPlanned {
			for _, c := range g.Desired {
				committedDesired[c.Key] = c.Value
			}
		}
		statusByName[g.Name] = res.Status
		results = append(results, res)
	}
	return results
}

// contradicts reports whether any desired condition sets a key to a
// different value than an already-committed goal requires.
func contradicts(desired []Condition, committed map[string]Value) bool {
	for _, c := range desired {
		if v, ok := committed[c.Key]; ok && v != c.Value {
			return true
		}
	}
	return false
}
