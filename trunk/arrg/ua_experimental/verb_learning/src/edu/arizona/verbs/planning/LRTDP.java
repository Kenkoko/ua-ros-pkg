package edu.arizona.verbs.planning;

import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.Stack;
import java.util.TreeMap;
import java.util.TreeSet;
import java.util.Vector;

import org.apache.log4j.Logger;

import ros.pkg.oomdp_msgs.msg.MDPState;
import ros.pkg.verb_learning.msg.Policy;

import com.google.common.base.Function;
import com.google.common.collect.Collections2;

import edu.arizona.verbs.Verb;
import edu.arizona.verbs.environment.Environment;
import edu.arizona.verbs.fsm.FSMState;
import edu.arizona.verbs.fsm.StateSet;
import edu.arizona.verbs.fsm.FSMState.StateType;
import edu.arizona.verbs.fsm.VerbFSM.TransitionResult;
import edu.arizona.verbs.mdp.OOMDPState;
import edu.arizona.verbs.mdp.StateConverter;
import edu.arizona.verbs.util.ProbUtils;

public class LRTDP {
	private static Logger logger = Logger.getLogger(LRTDP.class);
	
	public class SimulationResult {
		public State state;
		public Action action;
		public State nextState;
		public double cost;
	}

	public static final String terminateAction = "TERMINATE";
	
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
		public TreeSet<FSMState> fsmState_;
		double value_;
		boolean solved;
		private String hashString_;
		
		Map<Action, Map<State, Double>> transitions_ = new HashMap<LRTDP.Action, Map<State,Double>>();
		
		public State(OOMDPState mdpState, TreeSet<FSMState> fsmState) {
			mdpState_ = mdpState;
			fsmState_ = fsmState;
			solved = (isGoal() || isBadTerminal()); // Goal state are automatically solved
			
			// This is the initialization of the value function from the heuristic
			value_ = LRTDP.this.verb_.getFSM().getHeuristic(fsmState);
			
			hashString_ = fsmState_.toString() + "|" + mdpState_.toString();
			
//			for (Action a : LRTDP.this.actions_) {
//				transitions_.put(a, new HashMap<LRTDP.State, Double>());
//			}
		}
		
		public OOMDPState getMdpState() {
			return mdpState_;
		}
		
		public TreeSet<FSMState> getFsmState() {
			return fsmState_;
		}
		
		public boolean isGoal() { 
			return StateSet.getStateType(fsmState_).equals(StateType.GOOD_TERMINAL);
		}
		
		public boolean isBadTerminal() {
			return StateSet.getStateType(fsmState_).equals(StateType.BAD_TERMINAL);
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
		public Vector<Action> getGreedyActions() { 
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
		
//		public Action getGreedySolvedAction() {
//			List<Action> greedyActions = getGreedyActions();
//			for (Action action : greedyActions) {
//				
//			}
//		}
		
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
		
		// Stochastically simulate the next state
		public State pickNextState(Action a) {
			// This is an estimate distribution
			
//			Map<State, Double> nextStateDist = getNextStateDist(a);
//			State nextState = ProbUtils.sample(nextStateDist);
			
			// "Sampling" from the environment directly means the sample represents the true dist
			SimulationResult simulationResult = sampleNextState(this, a);
			updateMaps(simulationResult);
			
			return simulationResult.nextState;
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
			// If there is nothing, simulate once to get the ball rolling
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
	
	// BEGIN LRTDP CLASS
	
	public Verb verb_;
//	public State start_;
	public Environment environment_;
	public List<Action> actions_;
	
	public Map<String, State> knownStates_ = new HashMap<String, State>();
	
	public TreeMap<Action,HashMap<State,HashMap<State,Integer>>> transitionCounts_;
	public TreeMap<Action,HashMap<State,Double>> costTotals_;
	public TreeMap<Action,HashMap<State,Integer>> costCounts_;
	
	public LRTDP(Verb verb, Environment environment) {
		verb_ = verb;
		environment_ = environment;

		// This is so the entities have the remapped names
		// TODO: There's probably more ugliness coming with this name mapping, need to clean up
		// TODO: Move this out to the perform verb
		//environment_.initializeEnvironment(startState.getObjectStates());
		
		// Get the set of actions from the environment
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
	
	// This now returns the action to take from the start state, or null if the planning fails altogether
	public String runAlgorithm(OOMDPState startState, TreeSet<FSMState> fsmState) {
		logger.info("======================================\nBEGIN RTDP");
		
		// We have to do something like this because the first state might have relations that matter
		// TODO: Is this really right? Seems a little bit fudge-y. TURNING OFF FOR NOW
//		TransitionResult simulationResult = verb_.getFSM().simulateDfaTransition(fsmState, startState.getActiveRelations());
		State start = new State(startState, fsmState);
		if (knownStates_.containsKey(start.toString())) {
			start = knownStates_.get(start);
		} else {
			knownStates_.put(start.toString(), start);
		}
		
		logger.info("Start state is: " + start);
		
		if (start.isGoal()) { // Need to have a way to signal termination
			return terminateAction;
		}
		
		long startTime = System.currentTimeMillis();
		
		int i = 0;
		while (!start.solved) {
			long trialStart = System.currentTimeMillis();
			logger.info(">>>>>>>>>> BEGIN TRIAL " + i + ":");
			boolean success = runLrtdpTrial(start, 0.1);
			logger.info(">>>>>>>>>> END TRIAL " + i + ". Trial took " + 
						((System.currentTimeMillis() - trialStart)/1000.0) + " seconds.");
			i++;
			
			if (!success) {
				return null;
			}
		}
		
		logger.info("END RTDP. TOTAL TIME: " + ((System.currentTimeMillis() - startTime)/1000.0));
		return start.getGreedyActions().firstElement().name_;
	}

//	public String getBestAction(OOMDPState worldState, TreeSet<FSMState> fsmState) {
//		State state = new State(worldState, fsmState);
//		
//		if (knownStates_.containsKey(state.toString())) {
//			state = knownStates_.get(state.toString());
//			if (state.solved) {
//				return state.getGreedyActions().firstElement().name_;
//			} else {
//				// Re-plan
//			}
//		} else {
//			// Re-plan
//		}
//		
//		runAlgorithm();
//		
//		// Step 1: Push state's relations through VFSM, get combined state
//		// Step 2: Lookup combined state, see if it is closed
//		// Step 3: If closed, return max action, else plan until it is closed
//	}
	
//	public Policy recoverPolicy(final Map<String, String> argumentMap) { 
//		Policy plan = new Policy();
//		
//		List<OOMDPState> states = new Vector<OOMDPState>();
//		List<String> actions = new Vector<String>();
//		logger.info(">>>>>>>>>> POLICY <<<<<<<<<<<<<<");
//		State s = start_;
//		
//		int i = 0;
//		while (!s.isGoal() ) {
//			if (i >= 25) {
//				logger.error("POLICY RETRIEVAL FAILED: Returning partial plan");
//				break;
//			}
//			
//			logger.info("STATE:  " + s);
//			states.add(s.getMdpState());
//			logger.info("IS CURRENT STATE SOLVED? " + s.solved);
//			
//			List<Action> greedyActions = s.getGreedyActions();
//			Action a = null;
//			for (Action candidate : greedyActions) {
//				State sPrime = s.getNextStateDist(candidate).keySet().iterator().next();
//				if (sPrime.solved) { // Take the first action that leads to a solved state
//					a = candidate;
//					break;
//				}
//			}
//
//			if (a == null) {
//				logger.error("NO ACTION LEADS TO A SOLVED STATE, choosing stupid action");
//				a = greedyActions.iterator().next();
//			} 
//			
//			actions.add(a.name_);
//			
//			s = s.getNextStateDist(a).keySet().iterator().next();
////			logger.info("IS NEXT STATE SOLVED? " + s.solved);
//			
//			i++;
//		}
//		
//		logger.info("FINAL STATE:  " + s); 
//		states.add(s.getMdpState()); // Add the final state
//		actions.add("TERMINATE"); // TODO: Should make a constant in the msg file
//		
//		// Populate the object
//		// Functional programming in Java is so ugly!
//		plan.states = Collections2.transform(states, new Function<OOMDPState, MDPState>() 
//						{ public MDPState apply(OOMDPState state) { return StateConverter.stateToMsg(state.remapState(argumentMap)); } } ).toArray(new MDPState[0]);
//		plan.actions = actions.toArray(new String[0]);
//		return plan;
//	}
	
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
			s.update();
			
			// stochastically simulate next state
			State oldState = s;
			s = s.pickNextState(a);
			System.out.println("S:  " + ((int) oldState.value()) + " " + oldState);
			System.out.println("A:  " + a);
			System.out.println("S': " + ((int) s.value()) + " " + s);
			System.out.println("NEW? " + !oldState.equals(s));
//			oldState.printQValues();
			"adf".charAt(0);
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
	
	public double getMinCost() {
		return 1.0;
	}
	
	public double getExpectedCost(State s, Action a) {
//		System.out.println("GETTING REWARD FOR " + s);
		if (!costTotals_.get(a).containsKey(s)) {
			s.getNextStateDist(a);
		}
		
		double expectedCost = costTotals_.get(a).get(s) / costCounts_.get(a).get(s);
		
		return expectedCost;
	}
	
	public SimulationResult sampleNextState(State s, Action a) {
		OOMDPState nextMdpState = environment_.simulateAction(s.mdpState_, a.name_);
		
		TransitionResult dfaResult = verb_.getFSM().simulateDfaTransition(s.fsmState_, nextMdpState.getActiveRelations());
		State nextState = new State(nextMdpState, dfaResult.newState);
		
		if (knownStates_.containsKey(nextState.toString())) {
			nextState = knownStates_.get(nextState.toString());
		} else {
			knownStates_.put(nextState.toString(), nextState);
		}
		
		SimulationResult result = new SimulationResult();
		result.state = s;
		result.action = a;
		result.nextState = nextState;
		result.cost = dfaResult.cost;
		
		return result;
	}
	
	public Map<State, Double> simulateNextState(State s, Action a) {
		SimulationResult simulationResult = sampleNextState(s, a);
		return updateMaps(simulationResult);
	}
	
	public Map<State, Double> updateMaps(SimulationResult simulationResult) {
		// This is where T and R get updated
		State s = simulationResult.state;
		Action a = simulationResult.action;
		
		HashMap<State, HashMap<State, Integer>> stateStateProb = transitionCounts_.get(a);
		
		if (costTotals_.get(a).containsKey(s)) {
			costTotals_.get(a).put(s, costTotals_.get(a).get(s) + simulationResult.cost);
			costCounts_.get(a).put(s, costCounts_.get(a).get(s) + 1);
		} else {
			costTotals_.get(a).put(s, simulationResult.cost);
			costCounts_.get(a).put(s, 1);
		}
		 
		if (stateStateProb.containsKey(s)) {
			HashMap<State, Integer> nextStateDist = stateStateProb.get(s);
			if (nextStateDist.containsKey(simulationResult.nextState)) {
				nextStateDist.put(simulationResult.nextState, nextStateDist.get(simulationResult.nextState) + 1);
			} else {
				nextStateDist.put(simulationResult.nextState, 1);
			}
			return ProbUtils.computeProbDist(nextStateDist);
			
		} else {
			HashMap<State, Integer> nextStateDist = new HashMap<LRTDP.State, Integer>();
			nextStateDist.put(simulationResult.nextState, 1);
			stateStateProb.put(s, nextStateDist);
			return ProbUtils.computeProbDist(nextStateDist);
		}
	}
}
