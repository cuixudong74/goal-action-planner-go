package planner

import (
	"sort"

	"github.com/cuixudong74/goal-action-planner-go/world"
)

// orderGoals sorts goals deterministically: higher priority first, ties
// broken by name. Goal map/slice ordering from callers never leaks in.
func orderGoals(goals []Goal) []Goal {
	out := make([]Goal, len(goals))
	copy(out, goals)
	sort.Slice(out, func(i, j int) bool {
		if out[i].Priority != out[j].Priority {
			return out[i].Priority > out[j].Priority
		}
		return out[i].Name < out[j].Name
	})
	return out
}

// resolveDependencies returns the eligible goals in deterministic order.
// A goal is eligible iff every goal it DependsOn is present and eligible.
// A dependency cycle is reported with the cycle path (sorted start) instead
// of being followed forever.
func resolveDependencies(goals []Goal) (eligible []Goal, cycle []string) {
	byName := map[string]Goal{}
	for _, g := range goals {
		byName[g.Name] = g
	}
	const (
		white = 0 // unvisited
		gray  = 1 // on stack
		black = 2 // fully resolved
	)
	color := map[string]int{}
	eligibleSet := map[string]bool{}

	var visit func(name string, stack []string) bool // false => cycle
	visit = func(name string, stack []string) bool {
		switch color[name] {
		case black:
			return true
		case gray:
			// found a cycle; record it starting from its smallest element
			start := 0
			for i, n := range stack {
				if n == name {
					start = i
					break
				}
			}
			cycle = append([]string{}, stack[start:]...)
			return false
		}
		g, ok := byName[name]
		if !ok {
			return true // missing dependency handled by caller of the depender
		}
		color[name] = gray
		stack = append(stack, name)
		okDeps := true
		deps := append([]string{}, g.DependsOn...)
		sort.Strings(deps)
		for _, d := range deps {
			if _, present := byName[d]; !present {
				okDeps = false // dependency not among given goals: ineligible
				continue
			}
			if !visit(d, stack) {
				return false
			}
			if !eligibleSet[d] {
				okDeps = false
			}
		}
		color[name] = black
		eligibleSet[name] = okDeps
		return true
	}

	for _, g := range orderGoals(goals) {
		if !visit(g.Name, nil) {
			return nil, cycle
		}
	}
	for _, g := range orderGoals(goals) {
		if eligibleSet[g.Name] {
			eligible = append(eligible, g)
		}
	}
	return eligible, nil
}

// namedCondition pairs a condition with the goal it came from.
type namedCondition struct {
	cond world.Condition
	goal string
}

// detectConflicts merges the conditions of all goals and reports pairwise
// contradictions per fact key. It works by computing the feasible integer
// interval [lo, hi] for each key plus Neq exclusions; an empty interval or
// an excluded singleton is a conflict. Deterministic: keys and conditions
// are processed in sorted order.
func detectConflicts(goals []Goal) []Conflict {
	byKey := map[string][]namedCondition{}
	for _, g := range goals {
		for _, c := range g.Conditions {
			byKey[c.Key] = append(byKey[c.Key], namedCondition{c, g.Name})
		}
	}
	keys := make([]string, 0, len(byKey))
	for k := range byKey {
		keys = append(keys, k)
	}
	sort.Strings(keys)

	const inf = int64(1) << 62
	var conflicts []Conflict
	for _, k := range keys {
		conds := byKey[k]
		sort.Slice(conds, func(i, j int) bool {
			if conds[i].cond.String() != conds[j].cond.String() {
				return conds[i].cond.String() < conds[j].cond.String()
			}
			return conds[i].goal < conds[j].goal
		})
		lo, hi := -inf, inf
		var loC, hiC namedCondition
		var neqs []namedCondition
		for _, nc := range conds {
			c := nc.cond
			switch c.Op {
			case world.OpEq:
				if c.Value > lo {
					lo, loC = c.Value, nc
				}
				if c.Value < hi {
					hi, hiC = c.Value, nc
				}
			case world.OpGt:
				if c.Value+1 > lo {
					lo, loC = c.Value+1, nc
				}
			case world.OpGte:
				if c.Value > lo {
					lo, loC = c.Value, nc
				}
			case world.OpLt:
				if c.Value-1 < hi {
					hi, hiC = c.Value-1, nc
				}
			case world.OpLte:
				if c.Value < hi {
					hi, hiC = c.Value, nc
				}
			case world.OpNeq:
				neqs = append(neqs, nc)
			}
		}
		if lo > hi {
			conflicts = append(conflicts, Conflict{Key: k, GoalA: loC.goal, GoalB: hiC.goal, CondA: loC.cond, CondB: hiC.cond})
			continue
		}
		if lo == hi {
			for _, n := range neqs {
				if n.cond.Value == lo {
					pinner := loC
					if pinner.cond.String() == "" {
						pinner = hiC
					}
					conflicts = append(conflicts, Conflict{Key: k, GoalA: pinner.goal, GoalB: n.goal, CondA: pinner.cond, CondB: n.cond})
				}
			}
		}
	}
	return conflicts
}

// mergeConditions concatenates the conditions of goals in deterministic
// (priority, name) order.
func mergeConditions(goals []Goal) []world.Condition {
	var out []world.Condition
	for _, g := range goals {
		out = append(out, g.Conditions...)
	}
	return out
}

// goalNames returns goal names in order.
func goalNames(goals []Goal) []string {
	n := make([]string, len(goals))
	for i, g := range goals {
		n[i] = g.Name
	}
	return n
}
