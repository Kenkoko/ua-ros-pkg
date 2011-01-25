package edu.arizona.verbs.planning.fsm;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Vector;

import edu.arizona.verbs.planning.data.SimulationResult;
import edu.arizona.verbs.planning.shared.Action;
import edu.arizona.verbs.planning.shared.Planner;
import edu.arizona.verbs.planning.state.PlanningState;
import edu.arizona.verbs.shared.Environment;
import edu.arizona.verbs.shared.OOMDPState;
import edu.arizona.verbs.verb.vfsm.FSMVerb;
import edu.arizona.verbs.verb.vfsm.VerbState;

public abstract class FSMPlanner implements Planner {

	protected FSMVerb verb_;
	protected Environment environment_;
	protected List<Action> actions_;
	
	protected Map<String, PlanningState> knownStates_ = new HashMap<String, PlanningState>();
	
	public FSMPlanner(FSMVerb verb, Environment environment) {
		verb_ = verb;
		environment_ = environment;

		// Get the set of actions from the environment
		actions_ = new Vector<Action>();
		for (String action : environment_.getActions()) {
			actions_.add(new Action(action));
		}
//		actions_ = Lists.newArrayList(new Action("person 1000")); //, new Action("person 0100"), new Action("person 0010"));
		System.out.println(actions_);
	}
	
	@Override
	public List<Action> getActions() {
		return actions_;
	}

	@Override
	public FSMVerb getVerb() {
		return verb_;
	}

	// This is simply a convenient way to sample an (s, a, s') from the simulation 
	public SimulationResult sampleNextState(PlanningState s, Action a) {
		OOMDPState nextMdpState = environment_.simulateAction(s.getMdpState(), a.toString());
		
		VerbState vs = verb_.fsmTransition(s.getVerbState(), nextMdpState.getActiveRelations());
		PlanningState nextState = lookupState(nextMdpState, vs);
		
		SimulationResult result = new SimulationResult();
		result.state = s;
		result.action = a;
		result.nextState = nextState;
		
		return result;
	}
	
	// This version does not use simulateAction, it uses perform! Dangerous
	public SimulationResult groundSampleNextState(PlanningState s, Action a) {
		OOMDPState nextMdpState = environment_.simulateAction(a.toString());
		
		VerbState vs = verb_.fsmTransition(s.getVerbState(), nextMdpState.getActiveRelations());
		PlanningState nextState = lookupState(nextMdpState, vs);
		
		SimulationResult result = new SimulationResult();
		result.state = s;
		result.action = a;
		result.nextState = nextState;
		
		return result;
	}

	// Ensure that each state is cached 
	public PlanningState lookupState(OOMDPState mdpState, VerbState verbState) {
		String stateString = PlanningState.makeStateString(mdpState, verbState);
		if (!knownStates_.containsKey(stateString)) {
			knownStates_.put(stateString, new PlanningState(mdpState, verbState, this));
		}
		return knownStates_.get(stateString);
	}
}
