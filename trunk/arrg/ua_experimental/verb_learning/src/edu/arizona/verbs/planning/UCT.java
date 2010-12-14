package edu.arizona.verbs.planning;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Random;
import java.util.TreeSet;

import org.apache.commons.collections15.bag.HashBag;

import com.google.common.base.Splitter;
import com.google.common.collect.Iterables;

import edu.arizona.verbs.fsm.FSMState;
import edu.arizona.verbs.planning.shared.AbstractPlanner;
import edu.arizona.verbs.planning.shared.Action;
import edu.arizona.verbs.planning.shared.Policy;
import edu.arizona.verbs.planning.shared.SimulationResult;
import edu.arizona.verbs.planning.shared.State;
import edu.arizona.verbs.shared.Environment;
import edu.arizona.verbs.shared.OOMDPState;
import edu.arizona.verbs.verb.Verb;

public class UCT extends AbstractPlanner {
	
	private static double gamma = 0.9;
	
	private HashMap<String, HashMap<String, Double>> q = new HashMap<String, HashMap<String, Double>>();
	// Map state+depth+action to counts of states
	private HashMap<String, HashBag<String>> t = new HashMap<String, HashBag<String>>();
	private HashBag<String> nsd = new HashBag<String>();
	private HashBag<String> nasd = new HashBag<String>();
	
	private int maxDepth;
	private Random r = new Random();
	
	public UCT(Verb verb, Environment environment, int maxDepth) {
		super(verb, environment);
		setMaxDepth(maxDepth);
	}

	public void setMaxDepth(int maxDepth) {
		this.maxDepth = maxDepth;
	}
	
	@Override
	public Policy runAlgorithm(OOMDPState startState, TreeSet<FSMState> fsmState) {
		// Reset all statistics
		q = new HashMap<String, HashMap<String,Double>>();
		t = new HashMap<String, HashBag<String>>();
		nsd = new HashBag<String>();
		nasd = new HashBag<String>();
		
		State start = lookupState(startState, fsmState);
		
		if (start.isGoal()) {
			return new Policy();
		}
		
		for (int i = 0; i < 400; i++) {
			environment_.reset();
			
			System.out.println(">>> BEGIN TRIAL " + i);
			
			uct(start, maxDepth);
		}
		
		return recoverPolicy(start);
	}

	public double uct(State s, int d) {
		
		if (s.isGoal()) {
			System.out.println("Trial Reached Goal!");
			for (Action a : actions_) {
				setQ(s, a.toString(), d, 0.0);  
			}
			return 0.0;
			
		} else if (d == 1) { 
			for (Action a : actions_) {
				setQ(s, a.toString(), d, s.getHeuristic()); // Hmm. 
			}
			return 1.0;
			
		} else {

			// Choose the action with minimum Q-value
			String sdString = lookupString(s, d);
			List<Action> bestActions = new ArrayList<Action>();
			double minQ = Double.MAX_VALUE;
			for (Action a : actions_) {
				double qValue = getQ(s, a.toString(), d);
				String nasdString = lookupString(s, a, d);
				if (nasd.getCount(nasdString) > 0) {
					qValue += Math.sqrt(2 * Math.log(nsd.getCount(sdString)) / nasd.getCount(nasdString));
				}
				if (qValue < minQ) {
					bestActions.clear();
					bestActions.add(a);
					minQ = qValue;
				} else if (qValue == minQ) {
					bestActions.add(a);
				}
			}
			
			Action aStar = bestActions.get(r.nextInt(bestActions.size()));
			
			// Sample next state from environment
			SimulationResult result = groundSampleNextState(s, aStar);
			State nextState = result.nextState;
			
			// Update T
			String tString = lookupString(s, aStar, d);
			if (!t.containsKey(tString)) {
				t.put(tString, new HashBag<String>());
			}
			t.get(tString).add(nextState.toString());
			
			// Compute v
			double cost = getCost(s, nextState);
			double v = cost + gamma * uct(nextState, d - 1);

			// Increment nsd
			nsd.add(sdString);
			
			// Increment nasd
			String nasdString = lookupString(s, aStar, d);
			nasd.add(nasdString);

			// Update Q value
			double oldQ = getQ(s, aStar.toString(), d);
			double nasdCount = nasd.getCount(nasdString);
			double newQ = (oldQ * (nasdCount - 1) + v) / nasdCount;
			setQ(s, aStar.toString(), d, newQ);
			
			return v;
		}		
	}
	
	public Policy recoverPolicy(State start) {
		List<State> states = new ArrayList<State>();
		List<Action> actions = new ArrayList<Action>();
		
		State current = start;
		for (int d = maxDepth; d > 0; d--) {
			states.add(current);

			if (current.isGoal()) {
				actions.add(Action.TERMINATE_ACTION);
				break;
			}
			
			String sdString = lookupString(current, d);
			List<Action> bestActions = new ArrayList<Action>();
			double minQ = Double.MAX_VALUE;
			for (Action a : actions_) {
				String nasdString = lookupString(current, a, d);
				double qValue = getQ(current, a.toString(), d);
//				if (nasd.getCount(nasdString) > 0) {
//					qValue += Math.sqrt(2 * Math.log(nsd.getCount(sdString)) / nasd.getCount(nasdString));
//				}
				if (qValue < minQ) {
					bestActions.clear();
					bestActions.add(a);
					minQ = qValue;
				} else if (qValue == minQ) {
					bestActions.add(a);
				}
				
				System.out.println(a + ": " + qValue);
			}
			
			Action aStar = bestActions.get(r.nextInt(bestActions.size()));
			actions.add(aStar);
			
			String lookupString = lookupString(current, aStar, d);
			
			if (!t.containsKey(lookupString)) {
				break; // Why is this happening?
			} else {
				HashBag<String> nextStateCounts = t.get(lookupString);
				int max = Integer.MIN_VALUE;
				String mostLikelyNextState = null;
				for (String s : nextStateCounts.uniqueSet()) {
					int count = nextStateCounts.getCount(s);
					if (count > max) {
						max = count;
						mostLikelyNextState = s;
					}
				}
	
				String stateString = Iterables.getLast(Splitter.on("|").split(mostLikelyNextState));
				current = knownStates_.get(stateString);
				System.out.println(current);
			}
		}
		
		return new Policy(states, actions);
	}
	
	
	public boolean hasQ(State s, String a, int d) {
		String lookupString = lookupString(s, d);
		if (q.containsKey(lookupString)) {
			return q.get(lookupString).containsKey(a);
		}
		
		return false;
	}
	
	public double getQ(State s, String a, int d) {
		String lookupString = lookupString(s, d);
		if (q.containsKey(lookupString)) {
			HashMap<String, Double> qTable = q.get(lookupString);
			return qTable.get(a);
		} else {
			// If this is a new state, initialize
			HashMap<String, Double> qTable = new HashMap<String, Double>();
			for (Action action : actions_) {
				qTable.put(action.toString(), 0.0); 
//				qTable.put(action.toString(), s.getHeuristic()); // TODO: Is it OK to use heuristic here? 
			}
			q.put(lookupString, qTable);
			return q.get(lookupString).get(a);
		}
	}
	
	public void setQ(State s, String a, int d, double newQ) {
		String lookupString = lookupString(s, d);
		if (q.containsKey(lookupString)) {
			HashMap<String,Double> qTable = q.get(lookupString);
			qTable.put(a, newQ);
		} else {
			HashMap<String,Double> qTable = new HashMap<String, Double>();
			qTable.put(a.toString(), newQ);
			q.put(lookupString, qTable);
		}
	}
	
	private String lookupString(State s, int d) {
		return String.valueOf(d) + "|" + s.toString();
	}
	
	private String lookupString(State s, Action a, int d) {
		return String.valueOf(d) + "|" + a.toString() + "|" + s.toString();
	}
	
	public String chooseRandomAction() {
		Random r = new Random();
		int index = r.nextInt(actions_.size());
		return actions_.get(index).toString();
	}
	
	public double getCost(State s, State next) {
//		return (gamma * next.getHeuristic()) - s.getHeuristic() + 1.0;
		return (gamma * next.getHeuristic()) - s.getHeuristic() + (next.isGoal() ? 0.0 : 1.0);
	}
}
