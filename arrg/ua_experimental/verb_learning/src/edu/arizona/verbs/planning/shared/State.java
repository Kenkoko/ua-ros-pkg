package edu.arizona.verbs.planning.shared;

import java.util.HashMap;
import java.util.TreeMap;
import java.util.TreeSet;

import edu.arizona.verbs.fsm.FSMState;
import edu.arizona.verbs.fsm.FSMState.StateType;
import edu.arizona.verbs.fsm.StateSet;
import edu.arizona.verbs.shared.OOMDPState;

public class State {
//	private static Logger logger = Logger.getLogger(State.class);
	
	protected Planner planner_;
	protected OOMDPState mdpState_;
	protected TreeSet<FSMState> fsmState_;
	protected double value_;
	protected double heuristic_;
	protected String hashString_;
	
	protected HashMap<Action, HashMap<State, Integer>> transitionCounts_ = new HashMap<Action, HashMap<State,Integer>>();
	protected HashMap<Action, HashMap<State, Double>> transitionProbs_ = new HashMap<Action, HashMap<State,Double>>();
	protected TreeMap<Action, Double> costTotals_ = new TreeMap<Action, Double>();
	
	public State(OOMDPState mdpState, TreeSet<FSMState> fsmState, Planner planner) {
		planner_ = planner;
		
		mdpState_ = mdpState;
		fsmState_ = fsmState;
		
		// This is the initialization of the value function from the heuristic
		heuristic_ = planner_.getVerb().getFSM().getHeuristic(fsmState);
		value_ = heuristic_;
		
		for (Action a : planner_.getActions()) {
			transitionCounts_.put(a, new HashMap<State, Integer>());
			transitionProbs_.put(a, new HashMap<State, Double>());
			costTotals_.put(a, 0.0);
		}
		
		hashString_ = fsmState_.toString() + mdpState_.toString();
	}
	
	public OOMDPState getMdpState() {
		return mdpState_;
	}
	
	public TreeSet<FSMState> getFsmState() {
		return fsmState_;
	}
	
	public double getValue() {
		return value_;
	}
	
	public double getHeuristic() {
		return heuristic_;
	}
	
	public boolean isGoal() { 
		return StateSet.getStateType(fsmState_).equals(StateType.GOOD_TERMINAL);
	}
	
	public boolean isBadTerminal() {
		return StateSet.getStateType(fsmState_).equals(StateType.BAD_TERMINAL);
	}
	
	@Override
	public String toString() {
		return hashString_;
	}
}
