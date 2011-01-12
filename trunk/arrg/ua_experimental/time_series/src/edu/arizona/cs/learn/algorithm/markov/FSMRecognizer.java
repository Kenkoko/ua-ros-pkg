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
	
	public boolean update(List<BPPNode> active, Set<String> activeProps, boolean autoReset) {
		List<BPPNode> turningOn = new ArrayList<BPPNode>();
		List<BPPNode> turningOff = new ArrayList<BPPNode>();

		for (BPPNode node : active) {
			boolean currentState = node.active(activeProps);

			if (!currentState) {
				turningOff.add(node);
			}

			for (Edge e : this._graph.getOutEdges(node)) {
				BPPNode next = (BPPNode) this._graph.getDest(e);
				if ((!next.active(activeProps))
						|| (turningOff.indexOf(next) != -1))
					continue;
				turningOn.add(next);
				next.setActiveDepth(Math.max(next.getActiveDepth(), node.getActiveDepth() + 1));
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
			if (!node.isFinal() && this._graph.getOutEdges(node).size() != 0) {
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
	
	
	public boolean test(List<Interval> intervals, int start, int end) {
		List<BPPNode> active = new ArrayList<BPPNode>();
		active.add(_startNode);

		for (int i = start; i < end; i++) {
			Set<String> props = new HashSet<String>();
			for (Interval interval : intervals) {
				if (interval.on(i)) 
					props.add(interval.name);
			}

			if (update(active, props)) {
				return true;
			}
		}

		return false;
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