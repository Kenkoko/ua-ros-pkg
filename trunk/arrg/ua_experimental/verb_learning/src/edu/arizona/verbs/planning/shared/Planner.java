package edu.arizona.verbs.planning.shared;

import java.util.List;
import java.util.TreeSet;

import edu.arizona.verbs.fsm.FSMState;
import edu.arizona.verbs.planning.state.LRTDPState;
import edu.arizona.verbs.shared.OOMDPState;
import edu.arizona.verbs.verb.Verb;

public interface Planner {
	
	public List<Action> getActions();
	
	public Verb getVerb();
	
	public Policy runAlgorithm(OOMDPState startState, TreeSet<FSMState> fsmState);

	public SimulationResult sampleNextState(State state, Action a);
	
}
