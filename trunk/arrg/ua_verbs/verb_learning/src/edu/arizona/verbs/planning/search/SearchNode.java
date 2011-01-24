package edu.arizona.verbs.planning.search;

import java.util.Collection;
import java.util.ConcurrentModificationException;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Vector;

import edu.arizona.verbs.planning.data.SimulationResult;
import edu.arizona.verbs.planning.fsm.SearchPlanner;
import edu.arizona.verbs.planning.shared.Action;
import edu.arizona.verbs.planning.state.PlanningState;

public class SearchNode implements Comparable<SearchNode> {
	private SearchPlanner planner_;
	
	public PlanningState state;
	public int depth; 
	private int g; // Depth is also the g, since each step is +1 depth and +1 cost
	private int h; // Admissible heuristic for steps to goal
	private int f;
	
	private boolean read = false;
	private boolean changed = false;
	
	public LinkedHashMap<Action, SearchNode> children; // Preserve order for determinism
	public SearchNode parent = null;
	
	public SearchNode(SearchPlanner p, PlanningState state, int depth, int h) {
		planner_ = p;
		this.state = state;
		this.depth = depth;
		g = depth; 
		this.h = h;
		
		f = depth + h; // Initial estimate of path length = length so far + heuristic (of length to go)
		
		children = new LinkedHashMap<Action, SearchNode>();
	}
	
	public int f() {
		return f;
	}
	
	public int g() {
		return g;
	}
	
	public int h() {
		return h;
	}
	
	public void setG(int newG) {
		g = newG;
		f = depth + h;
		changed = true;
	}
	
	public void populateChildren() {
		for (Action a : planner_.getActions()) {
			SimulationResult result = planner_.sampleNextState(state, a);
			
			SearchNode child = planner_.lookupNode(result.nextState, depth + 1);
			child.parent = this;
			
			children.put(a, child);
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
			
			if (child.f() < minScore) {
				bestChildren = new Vector<SearchNode>();
				bestChildren.add(child);
			} else if (child.f() == minScore) {
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
		System.out.println("\tScore: " + f);
	}
	
	@Override
	public int compareTo(SearchNode other) {
		if (read && changed) {
			throw new ConcurrentModificationException("You changed the values after you sorted the list!");
		}
		
		read = true;
		changed = false;
		return this.f - other.f; // DANGER: Do not change f after sorting the list, ordering will be invalid!
	}
}
