package edu.arizona.verbs.verb.vfsm.learner;

import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

import edu.arizona.cs.learn.algorithm.markov.FSMRecognizer;
import edu.arizona.cs.learn.timeseries.model.Signature;
import edu.arizona.cs.learn.timeseries.recognizer.Recognizer;
import edu.arizona.verbs.fsm.VerbFSM;
import edu.arizona.verbs.fsm.core.CorePath;
import edu.arizona.verbs.mdp.StateConverter;
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
		
//		System.out.println("\n\n\n\n");
//		seq.print();
//		System.out.println("\n\n\n\n");
		
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
		
//		System.out.println("CORE PATHS:");
//		for (CorePath cp : corePaths_) {
//			cp.print();
//		}
	}
	
	@Override
	public FSMRecognizer getRecognizer() {
		Signature signature = new Signature("corepath");
		
		for (CorePath cp : corePaths_) {
			signature.update(StateConverter.convertTrace(cp.toStateList(), new HashMap<String, String>()));
		}
		
		FSMRecognizer recognizer = Recognizer.cave.build("corepath", signature, 0, false);
		return recognizer;
	}

	@Override
	public boolean isReady() {
		return !corePaths_.isEmpty();
	}
}
