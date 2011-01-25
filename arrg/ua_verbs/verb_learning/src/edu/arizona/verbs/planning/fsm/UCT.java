package edu.arizona.verbs.planning.fsm;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Random;

import org.apache.commons.collections15.bag.HashBag;

import com.google.common.base.Splitter;
import com.google.common.collect.Iterables;

import edu.arizona.verbs.planning.data.PlanningReport;
import edu.arizona.verbs.planning.data.SimulationResult;
import edu.arizona.verbs.planning.shared.Action;
import edu.arizona.verbs.planning.shared.Policy;
import edu.arizona.verbs.planning.shared.Policy.PolicyType;
import edu.arizona.verbs.planning.state.PlanningState;
import edu.arizona.verbs.shared.Environment;
import edu.arizona.verbs.shared.OOMDPState;
import edu.arizona.verbs.verb.vfsm.FSMVerb;
import edu.arizona.verbs.verb.vfsm.VerbState;

public class UCT extends FSMPlanner {
	
	private static int maxIterations = 500;
	private static double gamma = 0.8; // 0.8 seemed to work for go, 0.9 for go-via
	
	private HashMap<String, HashMap<String, Double>> q_ = new HashMap<String, HashMap<String, Double>>();
	// Map state+depth+action to counts of states
	private HashMap<String, HashBag<String>> t_ = new HashMap<String, HashBag<String>>();
	private HashBag<String> nsd_ = new HashBag<String>();
	private HashBag<String> nasd_ = new HashBag<String>();
	
	private int maxDepth_;
	private Random r_ = new Random();
	
	public UCT(FSMVerb verb, Environment environment, int maxDepth) {
		super(verb, environment);
		setMaxDepth(maxDepth);
	}
	
	public void setMaxDepth(int maxDepth) {
		this.maxDepth_ = maxDepth;
	}
	
	@Override
	public PlanningReport runAlgorithm(OOMDPState startState, VerbState verbState) {
		long startTime = System.currentTimeMillis();
		
		// Reset all statistics
		q_ = new HashMap<String, HashMap<String,Double>>();
		t_ = new HashMap<String, HashBag<String>>();
		nsd_ = new HashBag<String>();
		nasd_ = new HashBag<String>();
		
		PlanningState start = lookupState(startState, verbState);
		
		if (start.isTerminal()) {
			return new PlanningReport(new Policy(PolicyType.Terminate), true, (System.currentTimeMillis() - startTime));
		}

		boolean goDeep = false;
		
		goalCounter = 0;
		bestState = Integer.MAX_VALUE;
		int goalTotal = 0;
		for (int i = 1; i <= maxIterations; i++) {
			environment_.reset();
			
			if (i % 50 == 0) {
				System.out.println(">>> BEGIN TRIAL " + i + " (Reached Goal " + goalCounter + " of last 50 trials, got within " + bestState + " of goal)");
				
				goalTotal += goalCounter;
				goalCounter = 0;
				bestState = Integer.MAX_VALUE;

				if (goalTotal > 100) {
					goDeep = true;
					break;
				}
			}
			
			uct(start, maxDepth_);
		}
		
		return new PlanningReport(recoverPolicy(start, goDeep), true, (System.currentTimeMillis() - startTime));
		
//		return new PlanningReport(recoverPolicy(start, goalTotal > 100), true, (System.currentTimeMillis() - startTime));
//		return new PlanningReport(recoverPolicy(start, false), true, (System.currentTimeMillis() - startTime));
	}

	public static int goalCounter = 0;
	public static int bestState = Integer.MAX_VALUE;
	public double uct(PlanningState s, int d) {
		
//		System.out.println("STATE AT DEPTH " + d + ": " + s.getVerbState());
//		System.out.println("RELATIONS: " + s.getMdpState().getActiveRelations());
		bestState = Math.min(bestState, (int) s.getHeuristic());
		
		if (s.getVerbState().isGoodTerminal()) {
			goalCounter++;
			for (Action a : actions_) {
				setQ(s, a.toString(), d, 0.0);  
			}
			return 0.0;
		
		} else if (s.getVerbState().isBadTerminal()) {
			for (Action a : actions_) {
				setQ(s, a.toString(), d, s.getHeuristic()); // This should be tighter than using 0 
			}
			return s.getHeuristic(); // Should be something like 1000
			
		} else if (d == 0) { 
			for (Action a : actions_) {
				setQ(s, a.toString(), d, s.getHeuristic()); // This should be tighter than using 0 
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
				if (nasd_.getCount(nasdString) > 0) {
					qValue += Math.sqrt(Math.log(nsd_.getCount(sdString)) / nasd_.getCount(nasdString));
//					qValue += Math.sqrt(2 * Math.log(nsd_.getCount(sdString)) / nasd_.getCount(nasdString));
				}
				if (qValue < minQ) {
					bestActions.clear();
					bestActions.add(a);
					minQ = qValue;
				} else if (qValue == minQ) {
					bestActions.add(a);
				}
			}
			
			Action aStar = bestActions.get(r_.nextInt(bestActions.size()));
			
			// Sample next state from environment
			SimulationResult result = groundSampleNextState(s, aStar);
			PlanningState nextState = result.nextState;
			
			// Update T
			String tString = lookupString(s, aStar, d);
			if (!t_.containsKey(tString)) {
				t_.put(tString, new HashBag<String>());
			}
			t_.get(tString).add(nextState.toString());
			
			// Compute v
			double cost = getCost(s, nextState);
			double v = cost + gamma * uct(nextState, d - 1);

			// Increment nsd
			nsd_.add(sdString);
			
			// Increment nasd
			String nasdString = lookupString(s, aStar, d);
			nasd_.add(nasdString);

			// Update Q value
			
			double nasdCount = nasd_.getCount(nasdString);
			double oldQ = getQ(s, aStar.toString(), d);
			if (nasdCount == 1) {
				double newQ = v;
				setQ(s, aStar.toString(), d, newQ);
			} else {
				double newQ = (oldQ * (nasdCount - 1) + v) / nasdCount;
				setQ(s, aStar.toString(), d, newQ);
			}
			
			return v;
		}		
	}
	
	public Policy recoverPolicy(PlanningState start, boolean goDeep) { // return more than state ahead
		List<PlanningState> states = new ArrayList<PlanningState>();
		List<Action> actions = new ArrayList<Action>();

		PlanningState current = start;
		
		// This is the traditional UCT behavior
		if (!goDeep) {
			List<Action> bestActions = new ArrayList<Action>();
			double minQ = Double.MAX_VALUE;
			for (Action a : actions_) {
				double qValue = getQ(current, a.toString(), maxDepth_);
				
				if (qValue < minQ) {
					bestActions.clear();
					bestActions.add(a);
					minQ = qValue;
				} else if (qValue == minQ) {
					bestActions.add(a);
				}
				
//				System.out.println(a + ": " + qValue);
			}
			
			states.add(current);
			
			Action aStar = bestActions.get(r_.nextInt(bestActions.size()));
			actions.add(aStar);
			
			return new Policy(states, actions);
		}
		
		// Otherwise, cache a policy as deep as we can go while still having tried each action
		for (int d = maxDepth_; d > 0; d--) {
//			System.out.println("RECOVER POLICY CURRENT STATE: " + current.getVerbState());
			
			if (current.getVerbState().isGoodTerminal() || d == 0) {
//				System.out.println("TERMINAL REACHED! ");
				
				states.add(current);
				actions.add(Action.TERMINATE_ACTION);
				return new Policy(states, actions); 
			}
			
			List<Action> bestActions = new ArrayList<Action>();
			double minQ = Double.MAX_VALUE;
			for (Action a : actions_) {
				String nasdString = lookupString(current, a, d);
				double qValue = getQ(current, a.toString(), d);
				if (nasd_.getCount(nasdString) == 0 && d != maxDepth_) { // Must make a choice at the first level
					// Sparsity in the Q table means it's time to replan
					System.out.println("ZEROS IN Q TABLE: depth was " + d);
					return new Policy(states, actions); 
				}
				
				if (qValue < minQ) {
					bestActions.clear();
					bestActions.add(a);
					minQ = qValue;
				} else if (qValue == minQ) {
					bestActions.add(a);
				}
				
//				System.out.println(a + ": " + qValue);
			}
			
			states.add(current);
			
			Action aStar = bestActions.get(r_.nextInt(bestActions.size()));
			actions.add(aStar);
			
			String lookupString = lookupString(current, aStar, d);
			
			if (!t_.containsKey(lookupString)) {
				throw new RuntimeException("THIS HAPPENED");
//				break; // Is this still happening?
			} else {
				HashBag<String> nextStateCounts = t_.get(lookupString);
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
//				System.out.println(current);
			}
		}
		
		return new Policy(states, actions);
	}
	
	
	public boolean hasQ(PlanningState s, String a, int d) {
		String lookupString = lookupString(s, d);
		if (q_.containsKey(lookupString)) {
			return q_.get(lookupString).containsKey(a);
		}
		
		return false;
	}
	
	public double getQ(PlanningState s, String a, int d) {
		String lookupString = lookupString(s, d);
		if (q_.containsKey(lookupString)) {
			HashMap<String, Double> qTable = q_.get(lookupString);
			return qTable.get(a);
		} else {
			// If this is a new state, initialize
			HashMap<String, Double> qTable = new HashMap<String, Double>();
			for (Action action : actions_) {
				qTable.put(action.toString(), Double.NEGATIVE_INFINITY); 
			}
			q_.put(lookupString, qTable);
			return q_.get(lookupString).get(a);
		}
	}
	
	public void setQ(PlanningState s, String a, int d, double newQ) {
		String lookupString = lookupString(s, d);
		if (q_.containsKey(lookupString)) {
			HashMap<String,Double> qTable = q_.get(lookupString);
			qTable.put(a, newQ);
		} else {
			HashMap<String,Double> qTable = new HashMap<String, Double>();
			qTable.put(a.toString(), newQ);
			q_.put(lookupString, qTable);
		}
	}
	
	private String lookupString(PlanningState s, int d) {
		return String.valueOf(d) + "|" + s.toString();
	}
	
	private String lookupString(PlanningState s, Action a, int d) {
		return String.valueOf(d) + "|" + a.toString() + "|" + s.toString();
	}
	
	public String chooseRandomAction() {
		Random r = new Random();
		int index = r.nextInt(actions_.size());
		return actions_.get(index).toString();
	}
	
	public double getCost(PlanningState s, PlanningState next) {
//		return (gamma * next.getHeuristic()) - s.getHeuristic() + 1.0;
		
		double cost = (gamma * (1.0 * next.getHeuristic())) - (1.0 * s.getHeuristic()) + 1.0 * next.getVerbState().getCost();
//		System.out.println(s.getVerbState().posState + " --> " + next.getVerbState().posState + " = " + cost);
		return cost;
	}
}
