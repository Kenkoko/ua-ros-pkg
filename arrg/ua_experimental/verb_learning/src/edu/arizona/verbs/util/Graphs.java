package edu.arizona.verbs.util;

import java.util.Collection;
import java.util.HashSet;
import java.util.Set;
import java.util.Vector;

import edu.uci.ics.jung.graph.DirectedGraph;

// TODO: Some of this is redundant with StateMachines, probably can be merged into StateMachines
public class Graphs {
		
	public static<V, E> boolean isDFA(DirectedGraph<V, E> fsm) {
		for (V vertex : fsm.getVertices()) {
			Set<E> edgeSet = new HashSet<E>(); // This assumes the hash function valid
			for (E edge : fsm.getOutEdges(vertex)) {
				if (edgeSet.contains(edge)) {
					return false;
				}
				edgeSet.add(edge);
			}
		}
		
		return true;
	}

	public static<V, E> boolean hasEdge(DirectedGraph<V, E> graph, V source, E edge, V dest) {
		if (graph.getOutEdges(source).contains(edge)) {
			V opposite = graph.getOpposite(source, edge);
			return (opposite != null && opposite.equals(dest));
		} else {
			return false;
		}
	}
	
	public static<V, E> Collection<E> getEdgesBetweenVertices(V source, V dest, DirectedGraph<V, E> graph) { 
		Vector<E> result = new Vector<E>();
		for (E edge : graph.getOutEdges(source)) {
			if (graph.getOpposite(source, edge).equals(dest)) {
				result.add(edge);
			}
		}
		return result;
	}
	
	public static<V, E> Collection<V> getStartStates(DirectedGraph<V, E> graph) {
		Vector<V> result = new Vector<V>();
		for (V vertex : graph.getVertices()) {
			if (graph.getPredecessorCount(vertex) == 0) {
				result.add(vertex);
			}
		}
		return result;
	}
	
	public static<V, E> Collection<V> getTerminalStates(DirectedGraph<V, E> graph) {
		Vector<V> result = new Vector<V>();
		for (V vertex : graph.getVertices()) {
			if (graph.getSuccessorCount(vertex) == 0) {
				result.add(vertex);
			}
		}
		return result;
	}
	
	public static<V, E> boolean isEndState(DirectedGraph<V, E> fsm, V state) {
		return (fsm.getSuccessorCount(state) == 0);
	}
}
