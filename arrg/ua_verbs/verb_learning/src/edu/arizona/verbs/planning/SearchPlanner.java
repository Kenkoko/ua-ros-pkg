package edu.arizona.verbs.planning;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.PriorityQueue;

import org.apache.log4j.Logger;

import edu.arizona.verbs.planning.data.PlanningReport;
import edu.arizona.verbs.planning.search.SearchNode;
import edu.arizona.verbs.planning.shared.AbstractPlanner;
import edu.arizona.verbs.planning.shared.Policy;
import edu.arizona.verbs.planning.shared.Policy.PolicyType;
import edu.arizona.verbs.planning.state.PlanningState;
import edu.arizona.verbs.shared.Environment;
import edu.arizona.verbs.shared.OOMDPState;
import edu.arizona.verbs.verb.Verb;
import edu.arizona.verbs.verb.VerbState;

/*
 * An implementation of the A* search algorithm for planning with VFSMs
 */
public class SearchPlanner extends AbstractPlanner {

	private static Logger logger = Logger.getLogger(SearchPlanner.class);
	
	private SearchNode root_;
	private HashMap<String, SearchNode> knownNodes_ = new HashMap<String, SearchNode>();
	
	public SearchPlanner(Verb verb, Environment environment) {
		super(verb, environment);
		
		root_ = null;
	}
	
	@Override
	public PlanningReport runAlgorithm(OOMDPState startState, VerbState verbState) {
		long startTime = System.currentTimeMillis();
		
		PlanningState start = lookupState(startState, verbState);
		logger.info("Start state is: " + start);

		// Reset everything
		knownNodes_ = new HashMap<String, SearchNode>();
		root_ = lookupNode(start, 0);
		
		HashSet<SearchNode> closed = new HashSet<SearchNode>();
		PriorityQueue<SearchNode> open = new PriorityQueue<SearchNode>(); 
		open.add(root_);
		HashMap<SearchNode, SearchNode> cameFrom = new HashMap<SearchNode, SearchNode>();

		int visited = 0;
		int maxVisited = 10000;
		
		while (!open.isEmpty() && visited < maxVisited) { // Hack but needs to stop somehow if no solution 
			SearchNode x = open.remove();
			
			if (x.state.getVerbState().isGoodTerminal()) {
				List<SearchNode> path = reconstructPath(cameFrom, x);
				return new PlanningReport(new Policy(path), true, (System.currentTimeMillis() - startTime));
			}
			
			closed.add(x);
			
			for (SearchNode y : x.getChildren()) {
				if (closed.contains(y)) {
					continue;
				}
				
				int tentativeG = x.g() + 1;
				
				boolean tentativeIsBetter;
				boolean addToOpen = false;
				if (!open.contains(y)) {
					addToOpen = true;
					tentativeIsBetter = true;
				} else if (tentativeG < y.g()) {
					tentativeIsBetter = true;
				} else {
					tentativeIsBetter = false;
				}
				
				if (tentativeIsBetter) {
					cameFrom.put(y, x);
					y.setG(tentativeG);
				}
				
				if (addToOpen) { // Needed so that the value of F for y is already set when inserting into priority queue
					open.add(y);
				}
			}
			visited++;
		}
	
		return new PlanningReport(new Policy(PolicyType.Terminate), false, (System.currentTimeMillis() - startTime));
	}
	
	public List<SearchNode> reconstructPath(HashMap<SearchNode, SearchNode> cameFrom, SearchNode current) {
		if (cameFrom.containsKey(current)) {
			List<SearchNode> p = reconstructPath(cameFrom, cameFrom.get(current));
			p.add(current);
			return p;
		} else {
			List<SearchNode> justMe = new ArrayList<SearchNode>();
			justMe.add(current);
			return justMe;
		}
	}
	
	public SearchNode lookupNode(PlanningState s, int depth) {
		String hashString = String.valueOf(depth) + s.toString();
		if (!knownNodes_.containsKey(hashString)) {
			knownNodes_.put(hashString, new SearchNode(this, s, depth, getVerb().getHeuristic(s.getVerbState())));
		}
		
		return knownNodes_.get(hashString);
	}
	
	@Override
	public void setMaxDepth(int maxDepth) {
		// Could we use this for constraining the search?
	}
}
