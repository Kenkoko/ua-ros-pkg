package edu.arizona.verbs.planning.state;

import java.util.HashMap;
import java.util.Map;
import java.util.TreeMap;

import edu.arizona.verbs.planning.fsm.FSMPlanner;
import edu.arizona.verbs.planning.shared.Action;
import edu.arizona.verbs.shared.OOMDPState;
import edu.arizona.verbs.verb.vfsm.VerbState;

public class PlanningState {

	public static String makeStateString(OOMDPState mdpState, VerbState verbState) {
		return verbState.toString() + mdpState.toString();
	}
	
	protected FSMPlanner planner_;
	protected OOMDPState mdpState_;
	protected VerbState verbState_;
	protected double value_;
	protected int heuristic_;
	protected String hashString_;
	
	protected HashMap<Action, Map<PlanningState, Integer>> transitionCounts_ = new HashMap<Action, Map<PlanningState,Integer>>();
	protected HashMap<Action, Map<PlanningState, Double>> transitionProbs_ = new HashMap<Action, Map<PlanningState,Double>>();
	protected TreeMap<Action, Double> costTotals_ = new TreeMap<Action, Double>();
	
	public PlanningState(OOMDPState mdpState, VerbState verbState, FSMPlanner planner) {
		planner_ = planner;
		
		mdpState_ = mdpState;
		verbState_ = verbState;
		
		// This is the initialization of the value function from the heuristic
		heuristic_ = planner_.getVerb().getHeuristic(verbState_);
		value_ = heuristic_;
		
		for (Action a : planner_.getActions()) {
			transitionCounts_.put(a, new HashMap<PlanningState, Integer>());
			transitionProbs_.put(a, new HashMap<PlanningState, Double>());
			costTotals_.put(a, 0.0);
		}
		
		hashString_ = makeStateString(mdpState_, verbState_);
	}
	
//	public PlanningState(OOMDPState mdpState, Planner planner) {
//		planner_ = planner;
//		
//		mdpState_ = mdpState;
//		verbState_ = null;
//		
//		// This is the initialization of the value function from the heuristic
////		heuristic_ = planner_.getVerb().getHeuristic(verbState_);
////		value_ = heuristic_;
//		
//		heuristic_ = -1; // There is no heuristic
//		
//		for (Action a : planner_.getActions()) {
//			transitionCounts_.put(a, new HashMap<PlanningState, Integer>());
//			transitionProbs_.put(a, new HashMap<PlanningState, Double>());
//			costTotals_.put(a, 0.0);
//		}
//		
//		hashString_ = makeStateString(mdpState_, verbState_);
//	}
	
	public OOMDPState getMdpState() {
		return mdpState_;
	}
	
	public boolean hasVerbState() {
		return (verbState_ != null);
	}
	
	public VerbState getVerbState() {
		return verbState_;
	}
	
	public double getValue() {
		return value_;
	}
	
	public double getHeuristic() {
		return heuristic_;
	}
	
	public boolean isTerminal() { 
		return verbState_.isTerminal();
	}
	
	@Override
	public String toString() {
		return hashString_;
	}
}
