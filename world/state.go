// Package world defines the fact-based world state model shared by the
// planner and the runtime. A State is an immutable map from fact keys to
// int64 values; Conditions query facts and Effects mutate them.
package world

import (
	"fmt"
	"sort"
	"strings"
)

// Op is a comparison operator used in Conditions.
type Op int

const (
	OpEq Op = iota
	OpNeq
	OpLt
	OpLte
	OpGt
	OpGte
)

func (o Op) String() string {
	switch o {
	case OpEq:
		return "=="
	case OpNeq:
		return "!="
	case OpLt:
		return "<"
	case OpLte:
		return "<="
	case OpGt:
		return ">"
	case OpGte:
		return ">="
	}
	return "?"
}

// Condition is a predicate over a single fact key.
type Condition struct {
	Key   string
	Op    Op
	Value int64
}

// Check reports whether the condition holds in s. Missing keys read as 0.
func (c Condition) Check(s State) bool {
	v := s.Get(c.Key)
	switch c.Op {
	case OpEq:
		return v == c.Value
	case OpNeq:
		return v != c.Value
	case OpLt:
		return v < c.Value
	case OpLte:
		return v <= c.Value
	case OpGt:
		return v > c.Value
	case OpGte:
		return v >= c.Value
	}
	return false
}

func (c Condition) String() string { return fmt.Sprintf("%s%s%d", c.Key, c.Op, c.Value) }

// EffectOp is the mutation kind of an Effect.
type EffectOp int

const (
	EffSet EffectOp = iota // set fact to Value
	EffAdd                 // add Value to fact
)

// Effect mutates a single fact key.
type Effect struct {
	Key   string
	Op    EffectOp
	Value int64
}

// State is an immutable snapshot of world facts.
type State struct {
	facts map[string]int64
}

// NewState builds a State from the given facts (nil is allowed).
func NewState(facts map[string]int64) State {
	cp := make(map[string]int64, len(facts))
	for k, v := range facts {
		cp[k] = v
	}
	return State{facts: cp}
}

// Get returns the value of key, or 0 when absent.
func (s State) Get(key string) int64 { return s.facts[key] }

// With returns a copy of s with key set to v.
func (s State) With(key string, v int64) State {
	cp := make(map[string]int64, len(s.facts)+1)
	for k, val := range s.facts {
		cp[k] = val
	}
	cp[key] = v
	return State{facts: cp}
}

// Apply returns a copy of s with e applied.
func (s State) Apply(e Effect) State {
	switch e.Op {
	case EffSet:
		return s.With(e.Key, e.Value)
	case EffAdd:
		return s.With(e.Key, s.Get(e.Key)+e.Value)
	}
	return s
}

// ApplyAll returns a copy of s with all effects applied in order.
func (s State) ApplyAll(effs []Effect) State {
	for _, e := range effs {
		s = s.Apply(e)
	}
	return s
}

// Satisfies reports whether all conditions hold in s.
func (s State) Satisfies(conds []Condition) bool {
	for _, c := range conds {
		if !c.Check(s) {
			return false
		}
	}
	return true
}

// Keys returns the sorted fact keys. Iteration order is deterministic.
func (s State) Keys() []string {
	keys := make([]string, 0, len(s.facts))
	for k := range s.facts {
		keys = append(keys, k)
	}
	sort.Strings(keys)
	return keys
}

// Hash returns a canonical string encoding of the state, suitable as a
// map key for visited sets. Deterministic across runs.
func (s State) Hash() string {
	var b strings.Builder
	for _, k := range s.Keys() {
		fmt.Fprintf(&b, "%s=%d;", k, s.facts[k])
	}
	return b.String()
}

func (s State) String() string {
	var parts []string
	for _, k := range s.Keys() {
		parts = append(parts, fmt.Sprintf("%s=%d", k, s.facts[k]))
	}
	return "{" + strings.Join(parts, " ") + "}"
}
