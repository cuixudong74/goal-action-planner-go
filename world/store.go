package world

import "sync"

// Store is a versioned, concurrency-safe holder of the world state.
// Every successful mutation bumps the version, which lets planners and
// executors detect stale plans via optimistic concurrency checks.
type Store struct {
	mu      sync.Mutex
	state   State
	version uint64
}

// NewStore creates a Store with the given initial state.
func NewStore(initial State) *Store {
	return &Store{state: initial}
}

// Snapshot returns the current state and its version.
func (s *Store) Snapshot() (State, uint64) {
	s.mu.Lock()
	defer s.mu.Unlock()
	return s.state, s.version
}

// Version returns the current version.
func (s *Store) Version() uint64 {
	s.mu.Lock()
	defer s.mu.Unlock()
	return s.version
}

// ApplyResult reports the outcome of a conditional mutation.
type ApplyResult struct {
	OK           bool   // mutation applied
	Version      uint64 // version after the call (current version if !OK)
	StaleVersion bool   // expected version did not match
	FailedCond   string // first failing precondition, if any
}

// ApplyIf atomically checks preconditions and the expected version, then
// applies effects. The whole operation is all-or-nothing: a failed check
// leaves the state untouched.
func (s *Store) ApplyIf(expectedVersion uint64, pre []Condition, effs []Effect) ApplyResult {
	s.mu.Lock()
	defer s.mu.Unlock()
	if expectedVersion != s.version {
		return ApplyResult{Version: s.version, StaleVersion: true}
	}
	for _, c := range pre {
		if !c.Check(s.state) {
			return ApplyResult{Version: s.version, FailedCond: c.String()}
		}
	}
	s.state = s.state.ApplyAll(effs)
	s.version++
	return ApplyResult{OK: true, Version: s.version}
}

// Mutate unconditionally applies effects and returns the new version.
func (s *Store) Mutate(effs []Effect) uint64 {
	s.mu.Lock()
	defer s.mu.Unlock()
	s.state = s.state.ApplyAll(effs)
	s.version++
	return s.version
}
