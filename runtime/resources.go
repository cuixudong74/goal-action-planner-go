package runtime

// Resources tracks exclusive runtime resources. Because the scheduler
// processes agents in fixed ID order within each tick, contention outcomes
// are deterministic and every decision is logged with a reason.
type Resources struct {
	holders map[string]string // resource name -> holding agent ID
}

func newResources() *Resources {
	return &Resources{holders: map[string]string{}}
}

// acquire grants res to agent if free. The winner is whoever the
// deterministic schedule serves first; the loser gets a logged denial.
func (r *Resources) acquire(res, agent string) bool {
	if res == "" {
		return true
	}
	if _, held := r.holders[res]; held {
		return false
	}
	r.holders[res] = agent
	return true
}

// release frees res if held by agent. Releasing an unheld resource is a
// no-op, which keeps cancellation cleanup simple and total.
func (r *Resources) release(res, agent string) bool {
	if r.holders[res] == agent && res != "" {
		delete(r.holders, res)
		return true
	}
	return false
}

// releaseAll frees everything held by agent; used on cancel/timeout so no
// resource occupation is left dangling.
func (r *Resources) releaseAll(agent string) []string {
	var freed []string
	for res, holder := range r.holders {
		if holder == agent {
			freed = append(freed, res)
		}
	}
	for _, res := range freed {
		delete(r.holders, res)
	}
	return freed
}
