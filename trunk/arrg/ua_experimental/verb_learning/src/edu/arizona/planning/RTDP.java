package edu.arizona.planning;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Random;
import java.util.Stack;
import java.util.TreeMap;
import java.util.Vector;

import edu.arizona.environment.Environment;
import edu.arizona.planning.fsm.FSMState;
import edu.arizona.planning.fsm.FSMState.StateType;
import edu.arizona.planning.fsm.VerbDFA.SimulationResult;
import edu.arizona.planning.mdp.MDPState;
import edu.arizona.util.ProbUtils;
import edu.arizona.verbs.Verb;

public class RTDP {
	
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
		public MDPState mdpState_;
		public FSMState fsmState_;
		double value_;
		boolean solved;
		
		public State(MDPState mdpState, FSMState fsmState) {
			mdpState_ = mdpState;
			fsmState_ = fsmState;
			solved = isGoal(); // Goal state are automatically solved
			// This is the initialization of the value function from the heuristic
			if (solved) {
				value_ = 0.0; 
			} else {
				// TODO: Need to check error conditions
				value_ = RTDP.this.verb_.getDFA().getMinDistToGoodTerminal(fsmState);
			}
		}
		
		public MDPState getMdpState() {
			return mdpState_;
		}
		
		public FSMState getFsmState() {
			return fsmState_;
		}
		
		public boolean isGoal() { // TODO: What about bad goals?
			return fsmState_.getType().equals(StateType.GOOD_TERMINAL);
		}
		
		public Action greedyAction() { // Done.
			double minValue = Double.POSITIVE_INFINITY;
			Vector<Action> result = new Vector<Action>();
			for (Action a : RTDP.this.actions_) {
//				System.out.println("TESTING ACTION: " + a);
				double qValue = this.qValue(a);
				System.out.println("Action " + a + " has Q-value " + qValue);
				if (qValue < minValue) {
					minValue = qValue;
					result = new Vector<RTDP.Action>();
					result.add(a);
				} else if (qValue == minValue) {
					result.add(a);
				}
//				System.out.println(result);
			}
			
			if (result.size() > 1) {
//				Random generator = new Random();
//				int index = generator.nextInt(result.size());
//				return result.get(index);
				return result.firstElement();
			} else {
				return result.firstElement();
			}
		}
		
		public double qValue(Action a) { // Done?
//			return c(s, a) + Pa (s |s) · s .VALUE
			double q = RTDP.this.getExpectedCost(this, a);
			
			Map<State, Double> nextStateDist = getNextStateDist(this, a);
//			ProbUtils.printDist(nextStateDist);
			for (State next : nextStateDist.keySet()) {
				q += nextStateDist.get(next) * next.value_;
			}
			
			return q;
		}
		
		public void update() { // Done.
			Action a = this.greedyAction();
			this.value_ = this.qValue(a);
		}
		
		public State pickNextState(Action a) { 
			Map<State, Double> nextStateDist = getNextStateDist(this, a);
			State nextState = ProbUtils.sample(nextStateDist);
			return nextState;
		}
		
		public double residual() { // Done.
			Action a = this.greedyAction();
			double res = Math.abs(this.value() - this.qValue(a));
			System.out.println("RESIDUAL: " + res + " for state " + toString());
			return res;
		}
		
		public double value() {
			return value_;
		}

		@Override
		public String toString() {
			return fsmState_.toString() + "|" + mdpState_.toString();
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
	
	public RTDP(Verb verb, Environment environment, MDPState startState) {
		verb_ = verb;
		environment_ = environment;
		
		// We have to do something like this because the first state might have relations that matter
		SimulationResult simulationResult = verb_.getDFA().simulate(verb_.getDFA().getStartState(), startState.getActiveRelations());
		start_ = new State(startState, simulationResult.newState);
		
		System.out.println("START STATE is " + start_);
		
		actions_ = new Vector<RTDP.Action>();
		for (String action : environment_.getActions()) {
			actions_.add(new Action(action));
		}
		
		transitionCounts_ = new TreeMap<RTDP.Action, HashMap<State,HashMap<State,Integer>>>();
		costTotals_ = new TreeMap<RTDP.Action, HashMap<State,Double>>();
		costCounts_ = new TreeMap<RTDP.Action, HashMap<State,Integer>>();
		for (Action action : actions_) { 
			transitionCounts_.put(action, new HashMap<RTDP.State, HashMap<State,Integer>>());
			costTotals_.put(action, new HashMap<RTDP.State, Double>());
			costCounts_.put(action, new HashMap<RTDP.State, Integer>());
		}
	}
	
	public void runAlgorithm() {
//		while (s.residual() < 0.5) { // This is not the real condition
//		for (int i = 0; i < 5; i++) { // Nor, obviously, is this.
		int i = 0;
		while (!start_.solved) {
			System.out.println(">>>>>>>>>> BEGIN TRIAL " + i + ":");
			runLrtdpTrial(start_, 0.4);
			System.out.println(">>>>>>>>>> END TRIAL " + i + ":");
			i++;
		}
	}
	
	public void recoverPolicy() {
		List<MDPState> states = new Vector<MDPState>();
		System.out.println(">>>>>>>>>> POLICY <<<<<<<<<<<<<<");
		State s = start_;
		while (!s.isGoal()) {
			System.out.println("STATE:  " + s);
			Action a = s.greedyAction();
			System.out.println("ACTION: " + a);
			s = getNextStateDist(s, a).keySet().iterator().next();
		}
		System.out.println("FINAL STATE:  " + s);
	}
	
	public void runLrtdpTrial(State state, double epsilon) {
		State s = state;
		
		Stack<State> visited = new Stack<RTDP.State>();
		
		while (!s.solved) {
			// insert into visited
			visited.push(s);
			
			// check termination at goal states
			if (s.isGoal()) {
				break;
			}
			
			// pick best action and update hash
			Action a = s.greedyAction();
			System.out.println("ACTION: " + a);
			s.update();
			
			// stochastically simulate next state
			s = s.pickNextState(a);
			System.out.println(s);
		}
		
		// try labeling visited states in reverse order
		while (!visited.isEmpty()) {
			s = visited.pop();
			if (!checkSolved(s, epsilon)) {
				break;
			}
		}
	}
	
	public boolean checkSolved(State s, double epsilon) {
		boolean rv = true;
		Stack<State> open = new Stack<RTDP.State>();
		Stack<State> closed = new Stack<RTDP.State>();
		
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
			Map<State, Double> nextStateDist = getNextStateDist(s, a);
			for (State nextState : nextStateDist.keySet()) {
				if (!nextState.solved && !(open.contains(nextState) || closed.contains(nextState))) {
					open.push(nextState);
				}
			}
			
			if (rv) {
				// label relevant states
				for (State nextState : closed) {
					System.out.println("CLOSING STATE: " + nextState);
					nextState.solved = true;
				}
			} else {
				// update states with residuals and ancestors
				while (!closed.isEmpty()) {
					s = closed.pop();
					s.update();
				}
			}
		}
		
		return rv;
	}
	
	public void runRtdpTrial(State state) {
		State s = state;
		while (!s.isGoal()) {
			System.out.println("STATE IS: " + s);
			// pick best action and update hash
			Action a = s.greedyAction();
			System.out.println("GREEDY ACTION IS: " + a);
			s.update();
			System.out.println("NEW VALUE IS: " + s.value_);
			// stochastically simulate next state
			s = s.pickNextState(a);
			System.out.println("NEXT STATE IS: " + s);
			System.out.println("==========================================");
		}
	}
	
	public double getExpectedCost(State s, Action a) {
//		System.out.println("GETTING REWARD FOR " + s);
		if (!costTotals_.get(a).containsKey(s)) {
			getNextStateDist(s, a);
		}
		
		double expectedReward = costTotals_.get(a).get(s) / costCounts_.get(a).get(s);
		
//		System.out.println("Reward is " + expectedReward);
		
		return expectedReward;
	}
	
	public Map<State, Double> getNextStateDist(State s, Action a) {
		// Simulate to get the next state: This is where T and R get updated
		MDPState nextMdpState = environment_.simulateAction(s.mdpState_, a.name_);
		
		SimulationResult dfaResult = verb_.getDFA().simulate(s.fsmState_, nextMdpState.getActiveRelations());
		State nextState = new State(nextMdpState, dfaResult.newState);
		
		if (nextMdpState.isOutOfBounds()) {
			dfaResult.cost = 1000;
		}
		
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
			HashMap<State, Integer> nextStateDist = new HashMap<RTDP.State, Integer>();
			nextStateDist.put(nextState, 1);
			stateStateProb.put(s, nextStateDist);
			return ProbUtils.computeProbDist(nextStateDist);
		}
	}
}
