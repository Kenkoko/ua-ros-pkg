package edu.arizona.cs.learn.algorithm.markov;

import java.util.HashSet;
import java.util.Set;

import com.google.common.collect.HashMultimap;

import edu.arizona.cs.learn.util.graph.Edge;
import edu.uci.ics.jung.graph.DirectedGraph;

public class FSMUtil {

	public static DirectedGraph<BPPNode, Edge> composePrefix(
			DirectedGraph<BPPNode, Edge> activity, Set<BPPNode> actives,
			DirectedGraph<BPPNode, Edge> subActivity) {
		
		
		// Define fringe-edges = all edges that connect from an
		// active state to a non-active state.
		HashMultimap<Set<String>, BPPNode> fringeEdges = HashMultimap.create();
		for (BPPNode n : actives) {
			for (Edge e : activity.getOutEdges(n)) {
				BPPNode dest = activity.getDest(e);
				if (!actives.contains(dest)) {
					fringeEdges.put(e.props(), dest);
				}
			}
			
		}
		
		// Remove all active nodes & their associating incident edges
		for (BPPNode n : actives) {
			activity.removeVertex(n);
		}
				
				
		// Add nodes & edges from subActivity to activity
		Set<BPPNode> subEndNodes = new HashSet<BPPNode>();
		for (BPPNode n : subActivity.getVertices()) {
			activity.addVertex(n);
			if (n.isFinal() || subActivity.getOutEdges(n).size() == 0) {
				n.setIsFinal(false);
				subEndNodes.add(n);
			}
		}
		for (BPPNode n : subActivity.getVertices()) {
			for (Edge e : subActivity.getOutEdges(n)) {
				BPPNode dest = subActivity.getDest(e);
				activity.addEdge(e, n, dest);
			}
		}
		
		
		// Add fringe edges back to activity connecting each sub-end node
		// to a fringe in the activity
		for (BPPNode n : subEndNodes) {
			for (Set<String> props: fringeEdges.keySet()) {
				for (BPPNode dest : fringeEdges.get(props)) {
					Edge e = new Edge(props);
					activity.addEdge(e, n, dest);
					e.increment();
				}
			}
		}
		
		return activity;
	}
	
}
