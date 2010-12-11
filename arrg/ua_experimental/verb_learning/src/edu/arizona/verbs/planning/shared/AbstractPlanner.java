package edu.arizona.verbs.planning.shared;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.TreeSet;
import java.util.Vector;

import edu.arizona.verbs.fsm.FSMState;
import edu.arizona.verbs.fsm.VerbFSM.TransitionResult;
import edu.arizona.verbs.shared.Environment;
import edu.arizona.verbs.shared.OOMDPState;
import edu.arizona.verbs.verb.Verb;

public abstract class AbstractPlanner implements Planner {

	protected Verb verb_;
	protected Environment environment_;
	protected List<Action> actions_;
	
	protected Map<String, State> knownStates_ = new HashMap<String, State>();
	
	public AbstractPlanner(Verb verb, Environment environment) {
		verb_ = verb;
		environment_ = environment;

		// Get the set of actions from the environment
		actions_ = new Vector<Action>();
		for (String action : environment_.getActions()) {
			actions_.add(new Action(action));
		}
		System.out.println(actions_);
	}
	
	@Override
	public List<Action> getActions() {
		return actions_;
	}

	@Override
	public Verb getVerb() {
		return verb_;
	}

	// This is simply a convenient way to sample an (s, a, s') from the simulation 
	public SimulationResult sampleNextState(State s, Action a) {
		OOMDPState nextMdpState = environment_.simulateAction(s.getMdpState(), a.toString());
		
		TransitionResult dfaResult = verb_.getFSM().simulateDfaTransition(s.getFsmState(), nextMdpState.getActiveRelations());
		State nextState = lookupState(nextMdpState, dfaResult.newState);
		
		SimulationResult result = new SimulationResult();
		result.state = s;
		result.action = a;
		result.nextState = nextState;
		result.cost = dfaResult.cost;
		
		return result;
	}
	
	// This version does not use simulateAction, it uses perform! Dangerous
	public SimulationResult groundSampleNextState(State s, Action a) {
		OOMDPState nextMdpState = environment_.simulateAction(a.toString());
		
		TransitionResult dfaResult = verb_.getFSM().simulateDfaTransition(s.getFsmState(), nextMdpState.getActiveRelations());
		State nextState = lookupState(nextMdpState, dfaResult.newState);
		
		SimulationResult result = new SimulationResult();
		result.state = s;
		result.action = a;
		result.nextState = nextState;
		result.cost = dfaResult.cost;
		
		return result;
	}

	// Ensure that each state is cached 
	public State lookupState(OOMDPState mdpState, TreeSet<FSMState> fsmState) {
		String stateString = mdpState.toString() + fsmState.toString();
		if (!knownStates_.containsKey(stateString)) {
			knownStates_.put(stateString, new State(mdpState, fsmState, this));
		}
		return knownStates_.get(stateString);
	}
}
