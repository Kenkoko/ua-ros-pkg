package edu.arizona.verbs.planning;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Stack;

import org.apache.log4j.Logger;

import com.google.common.collect.Lists;

import edu.arizona.verbs.planning.data.PlanningReport;
import edu.arizona.verbs.planning.shared.AbstractPlanner;
import edu.arizona.verbs.planning.shared.Action;
import edu.arizona.verbs.planning.shared.Policy;
import edu.arizona.verbs.planning.shared.Policy.PolicyType;
import edu.arizona.verbs.planning.state.LRTDPState;
import edu.arizona.verbs.planning.state.PlanningState;
import edu.arizona.verbs.shared.Environment;
import edu.arizona.verbs.shared.OOMDPState;
import edu.arizona.verbs.verb.Verb;
import edu.arizona.verbs.verb.VerbState;

/*
 * An implementation of LRTDP (Labeled Real-Time Dynamic Programming) from (Bonet and Geffner, 2003)
 * adapted to VFSMs 
 */
public class LRTDP extends AbstractPlanner {
	
	public LRTDP(Verb verb, Environment environment) {
		super(verb, environment);
	}

	private static Logger logger = Logger.getLogger(LRTDP.class);
	
	// This now returns the action to take from the start state, or null if the planning fails altogether
	@Override
	public PlanningReport runAlgorithm(OOMDPState startState, VerbState verbState) {
		long startTime = System.currentTimeMillis();
//		logger.info("======================================\nBEGIN RTDP");
		
		double epsilon = 0.5;
		
		LRTDPState start = (LRTDPState) lookupState(startState, verbState);
		
		logger.info("Start state is: " + start);
		
		if (start.isTerminal()) { 
			return new PlanningReport(new Policy(PolicyType.Terminate), true, (System.currentTimeMillis() - startTime)); // TODO: We can do better than this if we bring back recoverPolicy
		}
		
		int i = 0;
		while (!start.isSolved()) {
			long trialStart = System.currentTimeMillis();
			logger.info(">>>>>>>>>> BEGIN TRIAL " + i + ":");
			boolean success = runLrtdpTrial(start, epsilon);
			logger.info(">>>>>>>>>> END TRIAL " + i + ". Trial was " + (success ? "" : "NOT") + 
						" successful and took " + ((System.currentTimeMillis() - trialStart)/1000.0) + 
						" seconds.");
			i++;
			
			if (!success) {
				return new PlanningReport(new Policy(PolicyType.Terminate), false, (System.currentTimeMillis() - startTime));
			}
		}
		
//		logger.info("END RTDP. TOTAL TIME: " + ((System.currentTimeMillis() - startTime)/1000.0));
		
		// TODO: LRTDP's recoverPolicy method was removed, need to restore it
		
		List<PlanningState> states = new ArrayList<PlanningState>();
		states.add(start);
		Policy policy = new Policy(states, Lists.newArrayList(start.getGreedyActions().firstElement()));
		return new PlanningReport(policy, true, (System.currentTimeMillis() - startTime));
	}
	
	public boolean runLrtdpTrial(LRTDPState state, double epsilon) {
		Stack<LRTDPState> visited = new Stack<LRTDPState>();
		
		boolean returnValue = true;
		
		LRTDPState s = state;
		while (!s.isSolved()) {
			// insert into visited
			visited.push(s);
			
			// check termination at goal states
			if (s.isTerminal()) {
				break;
			}
			
//			if (visited.size() > 20) { // Let's try a horizon on here
//				returnValue = true;
//				break;
//			}
			
			if (visited.size() > 1000) { // OLD
				return false;
			}
			
			// pick best action and update hash
			if (s.needsSimulaton()) { s.simulateAllActions(); }
			Action a = s.greedyAction();
			s.update();
			
			// stochastically simulate next state
//			LRTDPState oldState = s;
			s = s.pickNextState(a);
//			System.out.println("=====================================================");
//			System.out.println("S:  " + ((int) oldState.value()) + " " + oldState);
//			System.out.println("A:  " + a);
//			System.out.println("S': " + ((int) s.value()) + " " + s);
//			System.out.println("KNOWN STATES: " + knownStates_.size());
		}
		
		// try labeling visited states in reverse order
		while (!visited.isEmpty()) {
			s = visited.pop();
			if (!checkSolved(s, epsilon)) {
				break;
			}
		}
		
		return returnValue;
	}
	
	public boolean checkSolved(LRTDPState s, double epsilon) {
		boolean rv = true;
		Stack<LRTDPState> open = new Stack<LRTDPState>();
		Stack<LRTDPState> closed = new Stack<LRTDPState>();
		
		if (!s.isSolved()) {
			open.push(s);
		}
		
		while (!open.isEmpty()) {
			LRTDPState state = open.pop();
			closed.push(state);
			
			// check residual
			if (state.residual() > epsilon) {
				rv = false;
				continue;
			}
			
			// expand state
			Action a = state.greedyAction();
			Map<PlanningState, Double> nextStateDist = state.getNextStateDist(a);
			for (PlanningState nextishState : nextStateDist.keySet()) {
				LRTDPState nextState = (LRTDPState) nextishState;
				if (!nextState.isSolved() && !(open.contains(nextState) || closed.contains(nextState))) {
					if (nextState.needsSimulaton()) { nextState.simulateAllActions(); }
					open.push(nextState);
				}
			}
		}
			
		if (rv) { // label relevant states
			for (LRTDPState closedState : closed) {
				logger.info("CLOSING STATE: " + closedState);
				logger.info("CLOSED STATE HAS VALUE: " + closedState.getValue());
				closedState.printQValues();
				closedState.setSolved();
			}
		} else { // update states with residuals and ancestors
			while (!closed.isEmpty()) {
				LRTDPState notClosed = closed.pop();
				notClosed.update();
			}
		}
		
		return rv;
	}
	
	// Ensure that each state is cached
	@Override
	public PlanningState lookupState(OOMDPState mdpState, VerbState verbState) {
		String stateString = PlanningState.makeStateString(mdpState, verbState);
		if (!knownStates_.containsKey(stateString)) {
			knownStates_.put(stateString, new LRTDPState(mdpState, verbState, this));
		}
		return knownStates_.get(stateString);
	}

	@Override
	public void setMaxDepth(int maxDepth) {
		// Does this method make sense in LRTDP?
	}
}
