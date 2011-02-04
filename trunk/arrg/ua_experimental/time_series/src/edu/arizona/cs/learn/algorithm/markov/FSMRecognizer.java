package edu.arizona.cs.learn.algorithm.markov;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

import edu.arizona.cs.learn.timeseries.model.Interval;
import edu.arizona.cs.learn.util.graph.Edge;
import edu.uci.ics.jung.graph.DirectedGraph;

public class FSMRecognizer {
	private String _key;

	private List<BPPNode> _active;
	private DirectedGraph<BPPNode, Edge> _graph;
	private BPPNode _startNode;
	
	/** Timeout is a parameter that determines how long we can spend in an off state. */
	private int _timeOut = Integer.MAX_VALUE;

	public FSMRecognizer(String className, DirectedGraph<BPPNode, Edge> graph) {
		this._key = className;
		this._graph = graph;

		for (BPPNode n : graph.getVertices()) {
			if (n.label().equals("start")) {
				this._startNode = n;
			}
		}
		_active = new ArrayList<BPPNode>();
		_active.add(_startNode);
	}

	public String key() {
		return this._key;
	}

	public boolean update(Set<String> activeProps) {
		return update(this._active, activeProps);
	}

	public boolean update(Set<String> activeProps, boolean autoReset) {
		return update(this._active, activeProps, autoReset);
	}
	
	public boolean update(List<BPPNode> active, Set<String> activeProps) {
		return update(active, activeProps, true);
	}
	
	/**
	 * Update this recognizer with the new active state.  If this recognizer ends up in an 
	 * acceptance state, then return true, otherwise return false.
	 * @param active
	 * @param activeProps
	 * @param autoReset
	 * @return
	 */
	public boolean update(List<BPPNode> active, Set<String> activeProps, boolean autoReset) {
		return update(active, activeProps, autoReset, false);
	}
	
	/**
	 * Update this recognizer with the new active state.  If this recognizer ends up in an 
	 * acceptance state, then return true, otherwise return false.  If printDebug is true
	 * then lots and lots of debugging information is printed.
	 * @param active
	 * @param activeProps
	 * @param autoReset
	 * @param printDebug
	 * @return
	 */
	public boolean update(List<BPPNode> active, Set<String> activeProps, boolean autoReset, boolean printDebug) {
		List<BPPNode> turningOn = new ArrayList<BPPNode>();
		List<BPPNode> turningOff = new ArrayList<BPPNode>();
		
		if (printDebug) {
			System.out.println("  Update....");
			System.out.println("    Active Props: " + activeProps);
			System.out.println("    Active Nodes: " );
		}
		
		for (BPPNode node : active) {
			boolean currentState = node.active(activeProps);

			if (printDebug) { 
				System.out.println("     Node: " + node.id() + " -- " + node.getProps());
//				System.out.println("       Final? " + node.isFinal() + " -- " + _graph.getOutEdges(node).size());
//				System.out.println("       Active? " + currentState);
			}
			
			if (!currentState) {
				if (node.timeSpentOff() >= _timeOut)
					turningOff.add(node);
				
				if (printDebug) { 
//					System.out.println("     Node: " + node.id() + " -- " + node.getProps());
					System.out.println("       Turning Off!");
				}
			}

			for (Edge e : this._graph.getOutEdges(node)) {
				BPPNode next = (BPPNode) this._graph.getDest(e);
				if ((!next.active(activeProps))
						|| (turningOff.indexOf(next) != -1))
					continue;
				turningOn.add(next);
				next.setActiveDepth(Math.max(next.getActiveDepth(), node.getActiveDepth() + 1));
				
				if (printDebug) { 
					System.out.println("       Transitioning to node " + next.id());
				}
			}

		}

		for (BPPNode node : turningOff) {
			active.remove(node);
			node.setActiveDepth(0);
		}

		for (BPPNode node : turningOn) {
			if (active.indexOf(node) == -1) {
				active.add(node);
			}

		}

		for (BPPNode node : active) {
			if (!node.isFinal() && _graph.getOutEdges(node).size() != 0) {
				continue;
			}
			if (autoReset) {
				active.clear();
				active.add(this._startNode);
			}
			return true;
		}

		return false;
	}
	
	public boolean test(List<Interval> intervals) { 
		int start = Integer.MAX_VALUE;
		int end = 0;
		for (Interval interval : intervals) {
			start = Math.min(start, interval.start);
			end = Math.max(end, interval.end);
		}
		
		return test(intervals, start, end);
	}
	
	/**
	 * Returns true if this recognizer accepts the given episode represented
	 * as intervals.  Returns false otherwise.
	 * @param intervals
	 * @param start
	 * @param end
	 * @return
	 */
	public boolean test(List<Interval> intervals, int start, int end) {
		reset();

		for (int i = start; i < end; i++) {
			Set<String> props = new HashSet<String>();
			for (Interval interval : intervals) {
				if (interval.on(i)) 
					props.add(interval.name);
			}

			if (update(props)) {
				return true;
			}
		}

		return false;
	}
	
	/**
	 * Determine the depth, or how far along this recognizer is
	 * at every time step.
	 * @param intervals
	 * @param start
	 * @param end
	 * @param maxDepth -- filled in, must not be null
	 * @param maxDepthRatio -- filled in, must not be null
	 * @return whether or not the recognizer accepts
	 */
	public boolean test(List<Interval> intervals, int start, int end, 
			List<Double> maxDepth, List<Double> maxDepthRatio) { 
		reset();
		
		boolean accepted = false;
		List<BPPNode> actives = new ArrayList<BPPNode>();
		for (int j = start; j < end; j++) {
			Set<String> props = new HashSet<String>();
			for (Interval interval : intervals) {
				if (interval.on(j)) {
					props.add(interval.name);
				}
			}
			
			// Update each time step
			boolean accept = update(props, false);
			actives = getActive();
			
			// Find max depth among actives
			double maxActiveDepth = 0;
			double maxActiveDepthRatio = 0;
			for (BPPNode n : actives) {
				maxActiveDepth = Math.max(maxActiveDepth, n.getActiveDepth());

				double depth = n.getActiveDepth();
				double totalDepth = n.getActiveDepth() + n.getDistanceToFinal();
				maxActiveDepthRatio = Math.max(maxActiveDepthRatio, depth / totalDepth);
			}
			
			maxDepth.add(maxActiveDepth);
			maxDepthRatio.add(maxActiveDepthRatio);
			accepted = accept || accepted;
		}
		
		return accepted;
	}
	
	/**
	 * Lots of debugging information is printed as the given episode is
	 * traversed.
	 * @param intervals
	 * @return
	 */
	public boolean trace(List<Interval> intervals) {
		reset();
		
		int start = Integer.MAX_VALUE;
		int end = 0;
		for (Interval interval : intervals) {
			start = Math.min(start, interval.start);
			end = Math.max(end, interval.end);
		}
		
		boolean accept = false;
		for (int i = start; i < end; i++) {
			System.out.println(" TimeStep: " + i);
			Set<String> props = new HashSet<String>();
			for (Interval interval : intervals) {
				if (interval.on(i)) 
					props.add(interval.name);
			}
	
			boolean tmp = update(_active, props, false, true);
			if (tmp) { 
				System.out.println(".....Accepted");
			}
			accept = tmp || accept;
		}
		return accept;
	}	
	
	///////////////////////////////////////////////
	// Added by Daniel
	
	public List<BPPNode> getActive() {
		return _active;
	}
	
	public List<BPPNode> getActiveExceptStart() {
		List<BPPNode> result = new ArrayList<BPPNode>();
		result.addAll(_active);
		result.remove(_startNode);
		return result;
	}
	
	public BPPNode getStartState() {
		return _startNode;
	}
	
	public void reset() {
		_active = new ArrayList<BPPNode>();
		_active.add(this._startNode);
	}

	///////////////////////////////////////////////
	// Added by Anh
	
	public DirectedGraph<BPPNode, Edge> getGraph() {
		return _graph;
	}
	
}