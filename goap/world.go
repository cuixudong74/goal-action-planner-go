package goap

import (
	"errors"
	"sync"
)

// ErrVersionMismatch is returned by ApplyIfVersion when the world has
// changed since the caller's snapshot was taken.
var ErrVersionMismatch = errors.New("goap: world version mismatch")

// World is a concurrency-safe, versioned container of facts.
// Every successful mutation bumps the version, so planners and executors
// can detect that a snapshot they based decisions on has gone stale.
type World struct {
	mu      sync.Mutex
	state   State
	version uint64
}

// NewWorld creates a World holding a copy of the given initial state.
func NewWorld(initial State) *World {
	return &World{state: initial.Clone()}
}

// Snapshot returns a consistent copy of the state and its version.
func (w *World) Snapshot() (State, uint64) {
	w.mu.Lock()
	defer w.mu.Unlock()
	return w.state.Clone(), w.version
}

// Version returns the current version without copying the state.
func (w *World) Version() uint64 {
	w.mu.Lock()
	defer w.mu.Unlock()
	return w.version
}

// ApplyIfVersion atomically checks that the world is still at
// expectedVersion and, if so, applies the effects, returning the new
// version. Otherwise it returns ErrVersionMismatch and changes nothing.
func (w *World) ApplyIfVersion(effects []Effect, expectedVersion uint64) (uint64, error) {
	w.mu.Lock()
	defer w.mu.Unlock()
	if w.version != expectedVersion {
		return w.version, ErrVersionMismatch
	}
	for _, e := range effects {
		w.state[e.Key] = e.Value
	}
	w.version++
	return w.version, nil
}

// Apply unconditionally applies effects and returns the new version.
func (w *World) Apply(effects []Effect) uint64 {
	w.mu.Lock()
	defer w.mu.Unlock()
	for _, e := range effects {
		w.state[e.Key] = e.Value
	}
	w.version++
	return w.version
}
