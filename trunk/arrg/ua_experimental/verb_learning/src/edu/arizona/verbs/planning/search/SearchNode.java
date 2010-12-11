package edu.arizona.verbs.planning.search;

import java.util.Collection;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Vector;

import edu.arizona.verbs.planning.SearchPlanner;
import edu.arizona.verbs.planning.shared.Action;
import edu.arizona.verbs.planning.shared.SimulationResult;
import edu.arizona.verbs.planning.shared.State;

public class SearchNode implements Comparable<SearchNode> {
	private SearchPlanner planner_;
	
	public State state;
	public int depth; 
	public int g; // Depth is also the g, since each step is +1 depth and +1 cost
	public int h; // Admissible heuristic for steps to goal
	public int f;
	
	public LinkedHashMap<Action, SearchNode> children; // Preserve order for determinism
	public SearchNode parent = null;
	
	public SearchNode(SearchPlanner p, State state, int depth, int h) {
		planner_ = p;
		this.state = state;
		this.depth = depth;
		g = depth; 
		this.h = h;
		
		f = depth + h; // Initial estimate of path length = length so far + heuristic (of length to go)
		
		children = new LinkedHashMap<Action, SearchNode>();
	}
	
	public int getF() {
		return f;
	}
	
	public void populateChildren() {
		for (Action a : planner_.getActions()) {
			SimulationResult result = planner_.sampleNextState(state, a);
			
			SearchNode child = planner_.lookupNode(result.nextState, depth + 1);
			child.parent = this;
			
			children.put(a, child);
		}
	}
	
	public void updateParent() {
		if (parent != null) {
			if (parent.h < this.h + 1) {
				parent.h = this.h + 1;
			}
		}
	}
	
	public Collection<SearchNode> getChildren() {
		if (children.isEmpty()) { populateChildren(); }
		return children.values();
	}
	
	public List<SearchNode> getBestChildren() {
		if (children.isEmpty()) { populateChildren(); }
		
		int minScore = Integer.MAX_VALUE;
		List<SearchNode> bestChildren = new Vector<SearchNode>();
		
		for (Action a : planner_.getActions()) {
			SearchNode child = children.get(a);
			
			if (child.getF() < minScore) {
				bestChildren = new Vector<SearchNode>();
				bestChildren.add(child);
			} else if (child.getF() == minScore) {
				bestChildren.add(child);
			}
		}
		
		return bestChildren;
	}

	public void printNode() {
		System.out.println("==========================\nNode Info:");
		System.out.println("\tState: " + state);
		System.out.println("\tDepth: " + depth);
		System.out.println("\tHeuristic: " + h);
		System.out.println("\tScore: " + getF());
	}
	
	@Override
	public int compareTo(SearchNode other) {
		return this.f - other.f; // TODO: What if the score changes after sorting? How will it re-sort?
	}
}
