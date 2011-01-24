package edu.arizona.verbs.planning;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Random;
import java.util.Vector;

import org.apache.commons.collections15.bag.HashBag;

import edu.arizona.verbs.planning.shared.Action;
import edu.arizona.verbs.shared.Environment;
import edu.arizona.verbs.shared.OOMDPState;
import edu.arizona.verbs.verb.irl.IRLVerb;

public class SpecialUCT {

	protected IRLVerb verb_;
	protected Environment environment_;
	protected List<Action> actions_;
	
	protected Map<String, OOMDPState> knownStates_ = new HashMap<String, OOMDPState>();
	
	// UCT Parameters
	private static int maxIterations = 500;
	private static double gamma = 0.9; // 0.8 seemed to work for go
	private int maxDepth_;
	
	private HashMap<String, HashMap<String, Double>> q_ = new HashMap<String, HashMap<String, Double>>();
	// Map state+depth+action to counts of states
	private HashMap<String, HashBag<String>> t_ = new HashMap<String, HashBag<String>>();
	private HashBag<String> nsd_ = new HashBag<String>();
	private HashBag<String> nasd_ = new HashBag<String>();
	
	private Random r_ = new Random();
	
	public SpecialUCT(IRLVerb verb, Environment environment, int maxDepth) {
		verb_ = verb;
		environment_ = environment;
		actions_ = new Vector<Action>();
		for (String action : environment_.getActions()) {
			actions_.add(new Action(action));
		}
		setMaxDepth(maxDepth);
	}
	
	public void setMaxDepth(int maxDepth) {
		this.maxDepth_ = maxDepth;
	}
	
	public String runAlgorithm(OOMDPState startState) {
		// Reset all statistics
		q_ = new HashMap<String, HashMap<String,Double>>();
		t_ = new HashMap<String, HashBag<String>>();
		nsd_ = new HashBag<String>();
		nasd_ = new HashBag<String>();
		
		for (int i = 1; i <= maxIterations; i++) {
//			environment_.reset();
			
//			if (i % 50 == 0) {
//				System.out.println(">>> BEGIN TRIAL " + i);
//			}
			
			uct(startState, maxDepth_);
		}
		
		System.out.println("Q VALUES: ");
		List<Action> bestActions = new ArrayList<Action>();
		double maxQ = Double.NEGATIVE_INFINITY;
		for (Action a : actions_) {
			double qValue = getQ(startState, a.toString(), maxDepth_);
			
			if (qValue > maxQ) {
				bestActions.clear();
				bestActions.add(a);
				maxQ = qValue;
				
			} else if (qValue == maxQ) {
				bestActions.add(a);
			}
			
			System.out.println(a + ": " + qValue);
		}
		
		Action aStar = bestActions.get(r_.nextInt(bestActions.size()));
		
		return aStar.toString();
	}

	public double uct(OOMDPState s, int d) {
		if (d == 0) { 
			for (Action a : actions_) {
				setQ(s, a.toString(), d, 0.0); 
			}
			return 0.0; 
			
		} else {

			// Choose the action with minimum Q-value
			String sdString = lookupString(s, d);
			List<Action> bestActions = new ArrayList<Action>();
			double maxQ = Double.NEGATIVE_INFINITY;
			for (Action a : actions_) {
				double qValue = getQ(s, a.toString(), d);
				String nasdString = lookupString(s, a, d);
				if (nasd_.getCount(nasdString) > 0) {
					qValue += Math.sqrt(2 * Math.log(nsd_.getCount(sdString)) / nasd_.getCount(nasdString));
				}
				if (qValue > maxQ) {
					bestActions.clear();
					bestActions.add(a);
					maxQ = qValue;
				} else if (qValue == maxQ) {
					bestActions.add(a);
				}
			}
			
			Action aStar = bestActions.get(r_.nextInt(bestActions.size()));
			
			// Sample next state from environment
			OOMDPState nextState = sampleNextState(s, aStar);
			
			// Update T
			String tString = lookupString(s, aStar, d);
			if (!t_.containsKey(tString)) {
				t_.put(tString, new HashBag<String>());
			}
			t_.get(tString).add(nextState.toString());
			
			// Compute v
			double cost = verb_.getReward(s);
			double v = cost + gamma * uct(nextState, d - 1);

			// Increment nsd
			nsd_.add(sdString);
			
			// Increment nasd
			String nasdString = lookupString(s, aStar, d);
			nasd_.add(nasdString);

			// Update Q value
			double oldQ = getQ(s, aStar.toString(), d);
			double nasdCount = nasd_.getCount(nasdString);
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
	
	private OOMDPState sampleNextState(OOMDPState s, Action a) {
		OOMDPState nextMdpState = environment_.simulateAction(s, a.toString());
		return nextMdpState;
	}

	public boolean hasQ(OOMDPState s, String a, int d) {
		String lookupString = lookupString(s, d);
		if (q_.containsKey(lookupString)) {
			return q_.get(lookupString).containsKey(a);
		}
		
		return false;
	}
	
	public double getQ(OOMDPState s, String a, int d) {
		String lookupString = lookupString(s, d);
		if (q_.containsKey(lookupString)) {
			HashMap<String, Double> qTable = q_.get(lookupString);
			return qTable.get(a);
		} else {
			// If this is a new state, initialize
			HashMap<String, Double> qTable = new HashMap<String, Double>();
			for (Action action : actions_) {
				qTable.put(action.toString(), Double.POSITIVE_INFINITY); 
			}
			q_.put(lookupString, qTable);
			return q_.get(lookupString).get(a);
		}
	}
	
	public void setQ(OOMDPState s, String a, int d, double newQ) {
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
	
	private String lookupString(OOMDPState s, int d) {
		return String.valueOf(d) + "|" + s.toString();
	}
	
	private String lookupString(OOMDPState s, Action a, int d) {
		return String.valueOf(d) + "|" + a.toString() + "|" + s.toString();
	}
	
	public String chooseRandomAction() {
		Random r = new Random();
		int index = r.nextInt(actions_.size());
		return actions_.get(index).toString();
	}
}
