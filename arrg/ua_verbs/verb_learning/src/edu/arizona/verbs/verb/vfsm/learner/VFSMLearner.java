package edu.arizona.verbs.verb.vfsm.learner;

import java.util.List;

import edu.arizona.cs.learn.algorithm.markov.FSMRecognizer;
import edu.arizona.verbs.fsm.VerbFSM;
import edu.arizona.verbs.shared.OOMDPState;

public interface VFSMLearner {

	public void addTrace(List<OOMDPState> trace);	
	
	public void forgetTraces();
	
	public VerbFSM getFSM();
	
	public FSMRecognizer getRecognizer();
	
	public boolean isReady();
}
