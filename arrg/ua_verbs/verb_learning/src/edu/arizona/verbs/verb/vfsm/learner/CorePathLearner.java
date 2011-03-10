package edu.arizona.verbs.verb.vfsm.learner;

import java.util.HashSet;
import java.util.List;
import java.util.Set;

import edu.arizona.verbs.fsm.VerbFSM;
import edu.arizona.verbs.fsm.core.CorePath;
import edu.arizona.verbs.shared.OOMDPState;

public class CorePathLearner implements VFSMLearner {
	private Set<CorePath> corePaths_ = new HashSet<CorePath>();
	private VerbFSM vfsm_ = new VerbFSM();
	
	@Override
	public void addTrace(List<OOMDPState> trace) {
		updateCorePathSet(trace);
		vfsm_ = new VerbFSM(corePaths_);
	}

	@Override
	public void forgetTraces() {
		corePaths_ = new HashSet<CorePath>();
		vfsm_ = new VerbFSM();
	}
	
	@Override
	public VerbFSM getFSM() {
		return vfsm_;
	}
	
	// Utility method for updating core paths
	protected void updateCorePathSet(List<OOMDPState> trace) {
		CorePath seq = new CorePath(trace);
		
		boolean addPath = true;
		HashSet<CorePath> deadPaths = new HashSet<CorePath>();
		for (CorePath core : corePaths_) {
			if (core.contains(seq)) {
				deadPaths.add(core);
			} else if (seq.contains(core) || seq.equals(core)) {
				addPath = false;
			}
		}
		
		if (addPath) {
			corePaths_.add(seq);
		}
		
		for (CorePath dead : deadPaths) {
			corePaths_.remove(dead);
		}
		
		System.out.println("CORE PATHS:");
		for (CorePath cp : corePaths_) {
			cp.print();
		}
	}
}
