package planner

import (
	"container/heap"
	"strings"

	"github.com/cuixudong74/goal-action-planner-go/world"
)

// Options bounds the search so it can never run forever.
type Options struct {
	// MaxExpansions caps total expanded nodes across all Step calls.
	// Zero means DefaultMaxExpansions.
	MaxExpansions int
	// MaxDepth caps plan length. Zero means DefaultMaxDepth.
	MaxDepth int
}

const (
	DefaultMaxExpansions = 100000
	DefaultMaxDepth      = 64
)

func (o Options) withDefaults() Options {
	if o.MaxExpansions <= 0 {
		o.MaxExpansions = DefaultMaxExpansions
	}
	if o.MaxDepth <= 0 {
		o.MaxDepth = DefaultMaxDepth
	}
	return o
}

// node is one search frontier entry. seq uniquely identifies a path, which
// makes the queue key a total order and the search fully deterministic.
type node struct {
	state   world.State
	actions []*Action
	seq     string // action names joined by \x00
	cost    float64
	mutex   map[string]bool // MutexKeys already used along this path
}

// nodeHeap is a priority queue ordered by (cost, length, sequence).
type nodeHeap []*node

func (h nodeHeap) Len() int { return len(h) }
func (h nodeHeap) Less(i, j int) bool {
	a, b := h[i], h[j]
	if a.cost != b.cost {
		return a.cost < b.cost
	}
	if len(a.actions) != len(b.actions) {
		return len(a.actions) < len(b.actions)
	}
	return a.seq < b.seq
}
func (h nodeHeap) Swap(i, j int)       { h[i], h[j] = h[j], h[i] }
func (h *nodeHeap) Push(x interface{}) { *h = append(*h, x.(*node)) }
func (h *nodeHeap) Pop() interface{} {
	old := *h
	n := len(old)
	it := old[n-1]
	old[n-1] = nil
	*h = old[:n-1]
	return it
}

// bestKey orders closed-set entries the same way as the queue.
type bestKey struct {
	cost  float64
	depth int
	seq   string
}

func keyOf(n *node) bestKey { return bestKey{n.cost, len(n.actions), n.seq} }

func (k bestKey) leq(o bestKey) bool {
	if k.cost != o.cost {
		return k.cost < o.cost
	}
	if k.depth != o.depth {
		return k.depth < o.depth
	}
	return k.seq <= o.seq
}

// Search is a resumable, deterministic uniform-cost search. Call Step with
// a per-tick budget; a search that cannot finish within its budget reports
// StatusBudgetExhausted instead of blocking.
type Search struct {
	actions []*Action // sorted by name at construction
	goals   []world.Condition
	opts    Options

	open   nodeHeap
	closed map[string]bestKey // state hash -> best popped key

	expanded int
	done     bool
	result   Result
}

// NewSearch prepares a search from initial towards goals. The action slice
// is copied and sorted by name, so caller ordering never affects results.
func NewSearch(actions []*Action, initial world.State, goals []world.Condition, opts Options) *Search {
	sorted := make([]*Action, len(actions))
	copy(sorted, actions)
	sortActions(sorted)
	s := &Search{
		actions: sorted,
		goals:   goals,
		opts:    opts.withDefaults(),
		closed:  map[string]bestKey{},
	}
	root := &node{state: initial, mutex: map[string]bool{}}
	heap.Push(&s.open, root)
	return s
}

func sortActions(a []*Action) {
	for i := 1; i < len(a); i++ {
		for j := i; j > 0 && a[j].Name < a[j-1].Name; j-- {
			a[j], a[j-1] = a[j-1], a[j]
		}
	}
}

// Step expands at most budget nodes. It returns true once the search has
// finished (success or a definitive failure); call Result afterwards.
func (s *Search) Step(budget int) bool {
	if s.done {
		return true
	}
	for n := 0; n < budget; n++ {
		if len(s.open) == 0 {
			s.done = true
			s.result = Result{Status: StatusUnreachable, Expanded: s.expanded}
			return true
		}
		if s.expanded >= s.opts.MaxExpansions {
			s.done = true
			s.result = Result{Status: StatusBudgetExhausted, Expanded: s.expanded}
			return true
		}
		cur := heap.Pop(&s.open).(*node)
		h := cur.state.Hash()
		if best, seen := s.closed[h]; seen && best.leq(keyOf(cur)) {
			continue // already reached this state on an equal-or-better path
		}
		s.closed[h] = keyOf(cur)
		s.expanded++

		if cur.state.Satisfies(s.goals) {
			s.done = true
			s.result = Result{
				Status: StatusOK,
				Plan: Plan{
					Actions:   cur.actions,
					TotalCost: cur.cost,
				},
				Expanded: s.expanded,
			}
			return true
		}
		if len(cur.actions) >= s.opts.MaxDepth {
			continue
		}
		for _, a := range s.actions {
			if a.MutexKey != "" && cur.mutex[a.MutexKey] {
				continue // plan-level mutual exclusion
			}
			if !cur.state.Satisfies(a.Preconditions) {
				continue
			}
			next := &node{
				state:   cur.state.ApplyAll(a.Effects),
				actions: append(append([]*Action{}, cur.actions...), a),
				seq:     cur.seq + a.Name + "\x00",
				cost:    cur.cost + a.Cost,
			}
			next.mutex = make(map[string]bool, len(cur.mutex)+1)
			for k := range cur.mutex {
				next.mutex[k] = true
			}
			if a.MutexKey != "" {
				next.mutex[a.MutexKey] = true
			}
			heap.Push(&s.open, next)
		}
	}
	return false
}

// Result returns the final outcome. Only meaningful once Step returned true.
func (s *Search) Result() Result { return s.result }

// Expanded returns how many nodes have been expanded so far.
func (s *Search) Expanded() int { return s.expanded }

// seqString is a helper for tests and debugging.
func seqString(names []string) string { return strings.Join(names, ",") }
