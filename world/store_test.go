package world

import "testing"

func TestApplyIfIsAtomic(t *testing.T) {
	s := NewStore(NewState(map[string]int64{"a": 1}))
	pre := []Condition{{Key: "a", Op: OpEq, Value: 2}} // fails
	res := s.ApplyIf(0, pre, []Effect{{Key: "a", Op: EffSet, Value: 99}})
	if res.OK {
		t.Fatal("should have failed")
	}
	if res.FailedCond == "" {
		t.Fatal("expected failing condition to be reported")
	}
	st, v := s.Snapshot()
	if st.Get("a") != 1 || v != 0 {
		t.Fatalf("failed apply must not mutate: %v v=%d", st, v)
	}
}

func TestStaleVersionRejected(t *testing.T) {
	s := NewStore(NewState(nil))
	s.Mutate([]Effect{{Key: "x", Op: EffAdd, Value: 1}}) // version now 1
	res := s.ApplyIf(0, nil, []Effect{{Key: "y", Op: EffSet, Value: 1}})
	if res.OK || !res.StaleVersion {
		t.Fatalf("stale version must be rejected: %+v", res)
	}
	if _, v := s.Snapshot(); v != 1 {
		t.Fatalf("version moved: %d", v)
	}
}

func TestVersionBumpsOnSuccess(t *testing.T) {
	s := NewStore(NewState(nil))
	r1 := s.ApplyIf(0, nil, []Effect{{Key: "x", Op: EffSet, Value: 1}})
	r2 := s.ApplyIf(r1.Version, nil, []Effect{{Key: "x", Op: EffAdd, Value: 2}})
	if !r1.OK || !r2.OK || r2.Version != 2 {
		t.Fatalf("versions %d %d", r1.Version, r2.Version)
	}
	if st, _ := s.Snapshot(); st.Get("x") != 3 {
		t.Fatalf("x=%d", st.Get("x"))
	}
}

func TestStateHashDeterministic(t *testing.T) {
	a := NewState(map[string]int64{"k1": 1, "k2": 2})
	b := NewState(map[string]int64{"k2": 2, "k1": 1})
	if a.Hash() != b.Hash() {
		t.Fatal("hash must not depend on insertion order")
	}
}
