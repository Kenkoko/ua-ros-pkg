package edu.arizona.verbs.planning;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Random;

import org.apache.commons.collections15.bag.HashBag;

import com.google.common.base.Splitter;
import com.google.common.collect.Iterables;

import edu.arizona.verbs.planning.data.PlanningReport;
import edu.arizona.verbs.planning.data.SimulationResult;
import edu.arizona.verbs.planning.shared.AbstractPlanner;
import edu.arizona.verbs.planning.shared.Action;
import edu.arizona.verbs.planning.shared.Policy;
import edu.arizona.verbs.planning.shared.Policy.PolicyType;
import edu.arizona.verbs.planning.state.PlanningState;
import edu.arizona.verbs.shared.Environment;
import edu.arizona.verbs.shared.OOMDPState;
import edu.arizona.verbs.verb.vfsm.FSMVerb;
import edu.arizona.verbs.verb.vfsm.VerbState;

public class UCT extends AbstractPlanner {
	
	private static double gamma = 0.9;
	
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
		
		for (int i = 0; i < 400; i++) {
			environment_.reset();
			
			System.out.println(">>> BEGIN TRIAL " + i);
			
			uct(start, maxDepth_);
		}
		
		return new PlanningReport(recoverPolicy(start), true, (System.currentTimeMillis() - startTime));
	}

	public double uct(PlanningState s, int d) {
		
		if (s.getVerbState().isGoodTerminal()) {
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
				if (nasd_.getCount(nasdString) > 0) {
					qValue += Math.sqrt(2 * Math.log(nsd_.getCount(sdString)) / nasd_.getCount(nasdString));
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
			double oldQ = getQ(s, aStar.toString(), d);
			double nasdCount = nasd_.getCount(nasdString);
			double newQ = (oldQ * (nasdCount - 1) + v) / nasdCount;
			setQ(s, aStar.toString(), d, newQ);
			
			return v;
		}		
	}
	
	public Policy recoverPolicy(PlanningState start) {
		List<PlanningState> states = new ArrayList<PlanningState>();
		List<Action> actions = new ArrayList<Action>();
		
		PlanningState current = start;
		for (int d = maxDepth_; d > 0; d--) {
			states.add(current);

			if (current.getVerbState().isGoodTerminal()) {
				actions.add(Action.TERMINATE_ACTION);
				break;
			}
			
//			String sdString = lookupString(current, d);
			List<Action> bestActions = new ArrayList<Action>();
			double minQ = Double.MAX_VALUE;
			for (Action a : actions_) {
//				String nasdString = lookupString(current, a, d);
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
			// TODO: Maybe there's a better way to choose than randomly
			Action aStar = bestActions.get(r_.nextInt(bestActions.size()));
			actions.add(aStar);
			
			String lookupString = lookupString(current, aStar, d);
			
			if (!t_.containsKey(lookupString)) {
				break; // Why is this happening?
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
				System.out.println(current);
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
				qTable.put(action.toString(), 0.0); 
//				qTable.put(action.toString(), s.getHeuristic()); // TODO: Is it OK to use heuristic here? 
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
		return (gamma * next.getHeuristic()) - s.getHeuristic() + next.getVerbState().getCost();
	}
}
