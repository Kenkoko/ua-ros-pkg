package edu.arizona.planning;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Stack;
import java.util.TreeMap;
import java.util.Vector;

import org.apache.log4j.Logger;

import ros.pkg.oomdp_msgs.msg.MDPState;
import ros.pkg.verb_learning.msg.Policy;

import com.google.common.base.Function;
import com.google.common.collect.Collections2;

import edu.arizona.environment.Environment;
import edu.arizona.planning.fsm.FSMState;
import edu.arizona.planning.fsm.FSMState.StateType;
import edu.arizona.planning.fsm.VerbDFA.SimulationResult;
import edu.arizona.planning.mdp.OOMDPState;
import edu.arizona.util.ProbUtils;
import edu.arizona.verbs.Verb;

public class LRTDP {
	private static Logger logger = Logger.getLogger(LRTDP.class);
	
	// TODO: Maybe this is overkill
	public class Action implements Comparable<Action> {
		String name_;
		
		public Action(String name) {
			name_ = name;
		}
		
		@Override
		public boolean equals(Object arg0) {
			return (this.toString() == arg0.toString());
		}

		@Override
		public int hashCode() {
			return name_.hashCode();
		}

		@Override
		public String toString() {
			return name_;
		}

		@Override
		public int compareTo(Action arg0) {
			return name_.compareTo(arg0.name_);
		}
	}
	
	public class State {
		public OOMDPState mdpState_;
		public FSMState fsmState_;
		double value_;
		boolean solved;
		private String hashString_;
		
		Map<Action, Map<State, Double>> transitions_ = new HashMap<LRTDP.Action, Map<State,Double>>();
		
		public State(OOMDPState mdpState, FSMState fsmState) {
			mdpState_ = mdpState;
			fsmState_ = fsmState;
			solved = (isGoal() || isBadTerminal()); // Goal state are automatically solved
			// This is the initialization of the value function from the heuristic
			if (isGoal()) {
				solved = true;
				value_ = 0.0;
			} else if (isBadTerminal()) {
				solved = true;
				value_ = 1000; // BAD
			} else {
				solved = false;
				// Initializing the value to the heuristic
				value_ = LRTDP.this.verb_.getDFA().getHeuristic(fsmState);
			}
			
			// Actually, the value is always the same as the heuristic function
			
			hashString_ = fsmState_.toString() + "|" + mdpState_.toString();
		}
		
		public OOMDPState getMdpState() {
			return mdpState_;
		}
		
		public FSMState getFsmState() {
			return fsmState_;
		}
		
		public boolean isGoal() { 
			return fsmState_.getType().equals(StateType.GOOD_TERMINAL);
		}
		
		public boolean isBadTerminal() {
			return fsmState_.getType().equals(StateType.BAD_TERMINAL);
		}
		
		public Action greedyAction() { 
			List<Action> result = getGreedyActions();
			
			if (result.size() > 1) { // TODO: This random element messes things up, environment should be able to send over invalid states
//				Random generator = new Random();
//				int index = generator.nextInt(result.size());
//				return result.get(index);
				return result.iterator().next();
			} else {
				return result.iterator().next();
			}
		}
		
		// This is so we can have random for exploration, but a fixed policy at the end
		public List<Action> getGreedyActions() { 
			double minValue = Double.POSITIVE_INFINITY;
			Vector<Action> result = new Vector<Action>();
			for (Action a : LRTDP.this.actions_) {
//				System.out.println("TESTING ACTION: " + a);
				double qValue = this.qValue(a);
				if (qValue < minValue) {
					minValue = qValue;
					result = new Vector<LRTDP.Action>();
					result.add(a);
				} else if (qValue == minValue) {
					result.add(a);
				}
//				System.out.println(result);
			}
			
			return result;
		}
		
		public void printQValues() {
			for (Action a : LRTDP.this.actions_) {
				logger.info("Action " + a + " has Q-value " + this.qValue(a));
			}
		}
		
		// c(s,a) + sum_{s'} P(s'|s)*s'.value 
		public double qValue(Action a) { 
			double q = LRTDP.this.getExpectedCost(this, a); // c(s,a)

			// sum_{s'} P(s'|s)*s'.value
			Map<State, Double> nextStateDist = getNextStateDist(a);
			for (State next : nextStateDist.keySet()) {
				q += nextStateDist.get(next) * next.value_;
			}
			
			return q;
		}
		
		public void update() { 
			if (isGoal()) { System.err.println("WHY ARE YOU UPDATING A TERMINAL STATE!"); return; }
			
			Action a = this.greedyAction();
			this.value_ = this.qValue(a);
		}
		
		public State pickNextState(Action a) { 
			Map<State, Double> nextStateDist = getNextStateDist(a);
			State nextState = ProbUtils.sample(nextStateDist);
			return nextState;
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
			// NOTE: This is only for deterministic environments
			if (!transitions_.containsKey(a)) {
				transitions_.put(a, simulateNextState(this, a));
			}
			
			return transitions_.get(a);
		}

		@Override
		public String toString() {
			return hashString_;
		}

		@Override
		public boolean equals(Object arg0) {
			if (arg0 instanceof State) {
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
	
	// BEGIN RTDP CLASS
	
	public Verb verb_;
	public State start_;
	public Environment environment_;
	public List<Action> actions_;
	
	
	public TreeMap<Action,HashMap<State,HashMap<State,Integer>>> transitionCounts_;
	public TreeMap<Action,HashMap<State,Double>> costTotals_;
	public TreeMap<Action,HashMap<State,Integer>> costCounts_;
	
	public LRTDP(Verb verb, Environment environment, OOMDPState startState) {
		verb_ = verb;
		environment_ = environment;
		
		// We have to do something like this because the first state might have relations that matter
		SimulationResult simulationResult = verb_.getDFA().simulate(verb_.getDFA().getStartState(), startState.getActiveRelations());
		start_ = new State(startState, simulationResult.newState);
		
		System.out.println("START STATE is " + start_);
		
		actions_ = new Vector<LRTDP.Action>();
		for (String action : environment_.getActions()) {
			actions_.add(new Action(action));
		}
		
		transitionCounts_ = new TreeMap<LRTDP.Action, HashMap<State,HashMap<State,Integer>>>();
		costTotals_ = new TreeMap<LRTDP.Action, HashMap<State,Double>>();
		costCounts_ = new TreeMap<LRTDP.Action, HashMap<State,Integer>>();
		for (Action action : actions_) { 
			transitionCounts_.put(action, new HashMap<LRTDP.State, HashMap<State,Integer>>());
			costTotals_.put(action, new HashMap<LRTDP.State, Double>());
			costCounts_.put(action, new HashMap<LRTDP.State, Integer>());
		}
	}
	
	public boolean runAlgorithm() {
		logger.info("======================================\nBEGIN RTDP");
		logger.info("Start state is: " + start_);
		
		long start = System.currentTimeMillis();
		
		int i = 0;
		while (!start_.solved) {
			long trialStart = System.currentTimeMillis();
			logger.info(">>>>>>>>>> BEGIN TRIAL " + i + ":");
			boolean success = runLrtdpTrial(start_, 0.1);
			logger.info(">>>>>>>>>> END TRIAL " + i + ". Trial took " + 
						((System.currentTimeMillis() - trialStart)/1000.0) + " seconds.");
			i++;
			
			if (!success) {
				return false;
			}
			
//			if (((System.currentTimeMillis() - start) / 1000.0) > 60.0) {
//				return true;
//			}
		}
		
		logger.info("END RTDP. TOTAL TIME: " + ((System.currentTimeMillis() - start)/1000.0));
		return true;
	}
	
	public Policy recoverPolicy(final Map<String, String> argumentMap) { 
		Policy plan = new Policy();
		
		List<OOMDPState> states = new Vector<OOMDPState>();
		List<String> actions = new Vector<String>();
		logger.info(">>>>>>>>>> POLICY <<<<<<<<<<<<<<");
		State s = start_;
		
		int i = 0;
		while (!s.isGoal() ) {
			if (i >= 25) {
				logger.error("POLICY RETRIEVAL FAILED: Returning partial plan");
				break;
			}
			
			logger.info("STATE:  " + s);
			states.add(s.getMdpState());
			logger.info("IS CURRENT STATE SOLVED? " + s.solved);
			
			List<Action> greedyActions = s.getGreedyActions();
			Action a = null;
			for (Action candidate : greedyActions) {
				State sPrime = s.getNextStateDist(candidate).keySet().iterator().next();
				if (sPrime.solved) { // Take the first action that leads to a solved state
					a = candidate;
					break;
				}
			}

			if (a == null) {
				logger.error("NO ACTION LEADS TO A SOLVED STATE, choosing stupid action");
				a = greedyActions.iterator().next();
			} 
			
			actions.add(a.name_);
			
			s = s.getNextStateDist(a).keySet().iterator().next();
//			logger.info("IS NEXT STATE SOLVED? " + s.solved);
			
			i++;
		}
		
		logger.info("FINAL STATE:  " + s); 
		states.add(s.getMdpState()); // Add the final state
		actions.add("TERMINATE"); // TODO: Should make a constant in the msg file
		
		// Populate the object
		// Functional programming in Java is so ugly!
		plan.states = Collections2.transform(states, new Function<OOMDPState, MDPState>() 
						{ public MDPState apply(OOMDPState state) { return OOMDPState.remapState(state, argumentMap).convertToROS(); } } ).toArray(new MDPState[0]);
		plan.actions = actions.toArray(new String[0]);
		return plan;
	}
	
	public boolean runLrtdpTrial(State state, double epsilon) {
		State s = state;
		
		Stack<State> visited = new Stack<LRTDP.State>();
		
		while (!s.solved) {
			// insert into visited
			visited.push(s);
			
			// check termination at goal states
			if (s.isGoal()) {
				break;
			}
			
			if (visited.size() > 1000000) { // To prevent infinite planning
				return false;
			}
			
			// pick best action and update hash
			Action a = s.greedyAction();
//			System.out.println("ACTION: " + a);
			s.update();
			
			// stochastically simulate next state
			s = s.pickNextState(a);
//			System.out.println(s);
		}
		
		// try labeling visited states in reverse order
		while (!visited.isEmpty()) {
			s = visited.pop();
			if (!checkSolved(s, epsilon)) {
				break;
			}
		}
		
		return true;
	}
	
	public boolean checkSolved(State s, double epsilon) {
		boolean rv = true;
		Stack<State> open = new Stack<LRTDP.State>();
		Stack<State> closed = new Stack<LRTDP.State>();
		
		if (!s.solved) {
			open.push(s);
		}
		
		while (!open.isEmpty()) {
			s = open.pop();
			closed.push(s);
			
			// check residual
			if (s.residual() > epsilon) {
				rv = false;
				continue;
			}
			
			// expand state
			Action a = s.greedyAction();
			Map<State, Double> nextStateDist = s.getNextStateDist(a);
			for (State nextState : nextStateDist.keySet()) {
				if (!nextState.solved && !(open.contains(nextState) || closed.contains(nextState))) {
					open.push(nextState);
				}
			}
		}
			
		// NOTE: Did they really mean next state?
		if (rv) {
			// label relevant states
			for (State nextState : closed) {
				logger.info("CLOSING STATE: " + nextState);
				logger.info("CLOSED STATE HAS VALUE: " + nextState.value_);
				nextState.printQValues();
				nextState.solved = true;
			}
		} else {
			// update states with residuals and ancestors
			while (!closed.isEmpty()) {
				s = closed.pop();
				s.update();
			}
		}
		
		return rv;
	}
	
	public double getExpectedCost(State s, Action a) {
//		System.out.println("GETTING REWARD FOR " + s);
		if (!costTotals_.get(a).containsKey(s)) {
			s.getNextStateDist(a);
		}
		
		double expectedCost = costTotals_.get(a).get(s) / costCounts_.get(a).get(s);
		
		return expectedCost;
	}
	
	public Map<State, Double> simulateNextState(State s, Action a) {
		// Simulate to get the next state: This is where T and R get updated
		OOMDPState nextMdpState = environment_.simulateAction(s.mdpState_, a.name_);
		
		SimulationResult dfaResult = verb_.getDFA().simulate(s.fsmState_, nextMdpState.getActiveRelations());
		State nextState = new State(nextMdpState, dfaResult.newState);
		
		HashMap<State, HashMap<State, Integer>> stateStateProb = transitionCounts_.get(a);
		
		if (costTotals_.get(a).containsKey(s)) {
			costTotals_.get(a).put(s, costTotals_.get(a).get(s) + dfaResult.cost);
			costCounts_.get(a).put(s, costCounts_.get(a).get(s) + 1);
		} else {
			costTotals_.get(a).put(s, dfaResult.cost);
			costCounts_.get(a).put(s, 1);
		}
		 
		if (stateStateProb.containsKey(s)) {
			HashMap<State, Integer> nextStateDist = stateStateProb.get(s);
			if (nextStateDist.containsKey(nextState)) {
				nextStateDist.put(nextState, nextStateDist.get(nextState) + 1);
			} else {
				nextStateDist.put(nextState, 1);
			}
			return ProbUtils.computeProbDist(nextStateDist);
			
		} else {
			HashMap<State, Integer> nextStateDist = new HashMap<LRTDP.State, Integer>();
			nextStateDist.put(nextState, 1);
			stateStateProb.put(s, nextStateDist);
			return ProbUtils.computeProbDist(nextStateDist);
		}
	}
}
