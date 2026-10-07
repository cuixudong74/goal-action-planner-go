// Package goap implements a goal-oriented action planning and execution runtime.
package goap

import (
	"fmt"
	"sort"
)

// Value is a comparable fact value stored in the world state.
type Value = any

// State is an immutable-by-convention snapshot of world facts.
// Callers must not mutate a State obtained from Snapshot; use Clone.
type State map[string]Value

// Condition is an equality requirement on a single fact.
type Condition struct {
	Key   string
	Value Value
}

// Effect sets a single fact to a value when an action is applied.
type Effect struct {
	Key   string
	Value Value
}

// Clone returns a deep-enough copy (values must themselves be immutable).
func (s State) Clone() State {
	out := make(State, len(s))
	for k, v := range s {
		out[k] = v
	}
	return out
}

// SortedKeys returns the fact keys in deterministic (lexicographic) order.
func (s State) SortedKeys() []string {
	keys := make([]string, 0, len(s))
	for k := range s {
		keys = append(keys, k)
	}
	sort.Strings(keys)
	return keys
}

// Satisfies reports whether all conditions hold in the state.
func (s State) Satisfies(conds []Condition) bool {
	for _, c := range conds {
		v, ok := s[c.Key]
		if !ok || v != c.Value {
			return false
		}
	}
	return true
}

// Missing returns the conditions that do not hold in the state,
// in deterministic order.
func (s State) Missing(conds []Condition) []Condition {
	var out []Condition
	for _, c := range conds {
		v, ok := s[c.Key]
		if !ok || v != c.Value {
			out = append(out, c)
		}
	}
	return out
}

// Apply returns a new State with the effects applied; s is not modified.
func (s State) Apply(effects []Effect) State {
	out := s.Clone()
	for _, e := range effects {
		out[e.Key] = e.Value
	}
	return out
}

// Equal reports whether two states contain identical facts.
func (s State) Equal(o State) bool {
	if len(s) != len(o) {
		return false
	}
	for k, v := range s {
		ov, ok := o[k]
		if !ok || ov != v {
			return false
		}
	}
	return true
}

// String renders the state deterministically for logs and debugging.
func (s State) String() string {
	out := "{"
	for i, k := range s.SortedKeys() {
		if i > 0 {
			out += " "
		}
		out += fmt.Sprintf("%s=%v", k, s[k])
	}
	return out + "}"
}
