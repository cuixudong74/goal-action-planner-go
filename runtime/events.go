// Package runtime executes plans against a shared, versioned world store.
// It separates planning from execution, revalidates before every action,
// replans from interruption points, arbitrates shared resources
// deterministically, and supports cancellation, timeouts and resumption.
package runtime

import "fmt"

// EventKind classifies runtime log entries.
type EventKind int

const (
	EventPlanStarted EventKind = iota
	EventPlanned
	EventPlanInvalidated // world changed while planning; plan discarded
	EventActionStart
	EventActionApplied
	EventActionFailed   // injected or resource-starved failure
	EventPrecondBroken  // precondition no longer holds before apply
	EventEffectMismatch // postconditions violated after apply
	EventReplan         // replanning from the interruption point
	EventResourceAcquired
	EventResourceDenied
	EventResourceReleased
	EventGoalReached
	EventAgentFailed // replan budget exhausted or goal unreachable
	EventCancelled
	EventTimeout
)

func (k EventKind) String() string {
	names := []string{
		"plan-started", "planned", "plan-invalidated",
		"action-start", "action-applied", "action-failed",
		"precond-broken", "effect-mismatch", "replan",
		"resource-acquired", "resource-denied", "resource-released",
		"goal-reached", "agent-failed", "cancelled", "timeout",
	}
	if int(k) < len(names) {
		return names[k]
	}
	return "unknown"
}

// Event is one deterministic log entry. Tick is a logical clock; within a
// tick, entries appear in agent-ID order, so identical runs produce
// byte-identical logs.
type Event struct {
	Tick   uint64
	Agent  string
	Kind   EventKind
	Action string // action name, when relevant
	Detail string
}

func (e Event) String() string {
	s := fmt.Sprintf("t%d %s %s", e.Tick, e.Agent, e.Kind)
	if e.Action != "" {
		s += " " + e.Action
	}
	if e.Detail != "" {
		s += " (" + e.Detail + ")"
	}
	return s
}

// Log is an append-only event log.
type Log struct {
	events []Event
}

func (l *Log) append(e Event) { l.events = append(l.events, e) }

// Events returns a copy of all recorded events in order.
func (l *Log) Events() []Event {
	out := make([]Event, len(l.events))
	copy(out, l.events)
	return out
}

// String renders the log one event per line; ideal for golden comparisons.
func (l *Log) String() string {
	s := ""
	for _, e := range l.events {
		s += e.String() + "\n"
	}
	return s
}
