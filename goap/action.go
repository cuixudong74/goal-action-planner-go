package goap

import "sort"

// Action describes a deterministic state transition with a cost.
//
//   - Preconditions must all hold in the state where the action starts.
//   - Effects are applied atomically when the action completes.
//   - Mutex groups: at most one action per group may appear in a plan.
//   - Resources: scarce, named resources the action occupies while
//     executing; contention between agents is arbitrated by the Runtime.
type Action struct {
	Name          string
	Cost          float64
	Preconditions []Condition
	Effects       []Effect
	Mutex         []string
	Resources     []string
}

// Enabled reports whether the action's preconditions hold in s.
func (a *Action) Enabled(s State) bool {
	return s.Satisfies(a.Preconditions)
}

// EffectConditions views the effects as conditions, used to test
// whether the effects already hold in a state.
func (a *Action) EffectConditions() []Condition {
	out := make([]Condition, len(a.Effects))
	for i, e := range a.Effects {
		out[i] = Condition{Key: e.Key, Value: e.Value}
	}
	return out
}

// ActionSet is a deterministic container of actions keyed by name.
type ActionSet struct {
	byName map[string]*Action
	order  []string // sorted names
}

// NewActionSet validates and indexes the actions. Actions are always
// iterated in lexicographic name order so results never depend on the
// input slice order or map iteration order.
func NewActionSet(actions ...*Action) *ActionSet {
	s := &ActionSet{byName: make(map[string]*Action, len(actions))}
	for _, a := range actions {
		if a == nil || a.Name == "" {
			continue
		}
		if _, dup := s.byName[a.Name]; !dup {
			s.order = append(s.order, a.Name)
		}
		s.byName[a.Name] = a
	}
	sort.Strings(s.order)
	return s
}

// Get returns the action with the given name, or nil.
func (s *ActionSet) Get(name string) *Action { return s.byName[name] }

// All returns actions in deterministic (name-sorted) order.
func (s *ActionSet) All() []*Action {
	out := make([]*Action, 0, len(s.order))
	for _, n := range s.order {
		out = append(out, s.byName[n])
	}
	return out
}

// Len returns the number of distinct actions.
func (s *ActionSet) Len() int { return len(s.order) }
