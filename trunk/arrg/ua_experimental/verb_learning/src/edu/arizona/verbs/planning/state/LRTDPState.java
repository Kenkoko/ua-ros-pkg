package edu.arizona.verbs.planning.state;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Random;
import java.util.TreeMap;
import java.util.TreeSet;
import java.util.Vector;

import org.apache.log4j.Logger;

import edu.arizona.verbs.fsm.FSMState;
import edu.arizona.verbs.fsm.FSMState.StateType;
import edu.arizona.verbs.fsm.StateSet;
import edu.arizona.verbs.planning.shared.Action;
import edu.arizona.verbs.planning.shared.Planner;
import edu.arizona.verbs.planning.shared.SimulationResult;
import edu.arizona.verbs.planning.shared.State;
import edu.arizona.verbs.shared.OOMDPState;
import edu.arizona.verbs.util.ProbUtils;

public class LRTDPState extends State {
	
	private static Logger logger = Logger.getLogger(LRTDPState.class);
	
	private boolean solved;
	
	private int simulationCount_ = 0; // This is the number of times we have simulated all outgoing transitions from this state
	
	public LRTDPState(OOMDPState mdpState, TreeSet<FSMState> fsmState, Planner planner) {
		super(mdpState, fsmState, planner);
		
		solved = (isGoal() || isBadTerminal()); // Goal state are automatically solved
		
		hashString_ = fsmState_.toString() + mdpState_.toString();
	}
	
	public Action greedyAction() { 
		List<Action> result = getGreedyActions();
		
		// TODO: Add this random element back
		if (result.size() > 1) { 
//			Random generator = new Random();
//			int index = generator.nextInt(result.size());
//			return result.get(index);
			return result.iterator().next();
		} else {
			return result.iterator().next();
		}
	}
	
	public Vector<Action> getGreedyActions() { 
		double minValue = Double.POSITIVE_INFINITY;
		
		Vector<Action> result = new Vector<Action>();
		for (Action a : planner_.getActions()) {
			double qValue = this.qValue(a);
			if (qValue < minValue) {
				minValue = qValue;
				result = new Vector<Action>();
				result.add(a);
			} else if (qValue == minValue) {
				result.add(a);
			}
		}
		
		return result;
	}
	
	public void printQValues() {
		for (Action a : planner_.getActions()) {
			logger.info("Action " + a + " has Q-value " + this.qValue(a));
		}
	}
	
	// c(s,a) + sum_{s'} P(s'|s)*s'.value 
	public double qValue(Action a) { 
		double q = getExpectedCost(a); // c(s,a)

		// sum_{s'} P(s'|s)*s'.value
		Map<State, Double> nextStateDist = getNextStateDist(a);
		for (State nextish : nextStateDist.keySet()) {
			LRTDPState next = (LRTDPState) nextish;
			q += nextStateDist.get(next) * next.value_;
		}
		
		return q;
	}
	
	public void update() { 
		if (isGoal()) { System.err.println("WHY ARE YOU UPDATING A TERMINAL STATE!"); return; }
		
		Action a = this.greedyAction();
		this.value_ = this.qValue(a);
	}
	
	// Stochastically simulate the next state
	public LRTDPState pickNextState(Action a) {
		return (LRTDPState) ProbUtils.sample(transitionProbs_.get(a));
	}
	
	public double residual() {
		Action a = this.greedyAction();
		double res = Math.abs(this.value() - this.qValue(a));
		return res;
	}
	
	public double value() {
		return value_;
	}
	
	public Map<State,Double> getNextStateDist(Action a) {
		return transitionProbs_.get(a);
	}

	public boolean needsSimulaton() {
		return (simulationCount_ < 1); // This would be increased to behave more like SS 
	}
	
	public void simulateAllActions() {
		for (Action a : planner_.getActions()) {
			SimulationResult sr = planner_.sampleNextState(this, a);
			
			// Update transition counts
			int oldCount = 0;
			if (transitionCounts_.get(a).containsKey(sr.nextState)) {
				oldCount = transitionCounts_.get(a).get(sr.nextState);
			}
			transitionCounts_.get(a).put(sr.nextState, oldCount + 1);
			
			// Compute transition probabilities
			transitionProbs_.put(a, ProbUtils.computeProbDist(transitionCounts_.get(a)));
			
			double oldCostTotal = 0.0;
			if (costTotals_.containsKey(a)) {
				oldCostTotal = costTotals_.get(a);
			}
			costTotals_.put(a, oldCostTotal + sr.cost);		
		}
		
		simulationCount_++;
	}
	
	// If this is slow we can cache the total count per action as well
	public double getExpectedCost(Action a) {
//		HashMap<State, Integer> countMap = transitionCounts_.get(a); 
//		double totalCost = costTotals_.get(a);
//		
//		double totalCount = 0;
//		for (State nextState : countMap.keySet()) {
//			totalCount += countMap.get(nextState);
//		}
//		
//		return (totalCost / totalCount);
		
		return 1.0; // This is all this ever was anyway
	}
	
	public boolean isSolved() {
		return solved;
	}
	
	public void setSolved() {
		solved = true;
	}

	@Override
	public boolean equals(Object arg0) {
		if (arg0 instanceof LRTDPState) {
			return this.toString().equals(arg0.toString());
		} else {
			return false;
		}
	}

	@Override
	public int hashCode() {
		return toString().hashCode();
	}
}
