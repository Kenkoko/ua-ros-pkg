package edu.arizona.util;

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Set;

import edu.arizona.cs.learn.algorithm.markov.BPPNode;
import edu.arizona.cs.learn.util.graph.Edge;
import edu.arizona.planning.fsm.FSMState;
import edu.arizona.planning.fsm.FSMState.StateType;
import edu.uci.ics.jung.graph.DirectedGraph;
import edu.uci.ics.jung.graph.DirectedSparseGraph;

public class GraphUtils {
	
	public static DirectedGraph<FSMState, Edge> convertToFSM(DirectedGraph<BPPNode, Edge> graph) {
		DirectedGraph<FSMState, Edge> fsm = new DirectedSparseGraph<FSMState, Edge>();
		
		addSelfLoops(graph);
		
		HashMap<BPPNode, FSMState> nodeMap = new HashMap<BPPNode, FSMState>();
		for (BPPNode node : graph.getVertices()) {
			FSMState state = null;
			if (node.isStart()) {
				state = new FSMState(node, StateType.START);
			} else if (graph.getSuccessorCount(node) == 1) { // TODO: This is because we have already added the self-loops
				state = new FSMState(node, StateType.GOAL);
			} else {
				state = new FSMState(node, StateType.GOOD);
			}
			nodeMap.put(node, state);
		}
		
		for (Edge e : graph.getEdges()) {
			fsm.addEdge(new Edge(e.props()), nodeMap.get(graph.getSource(e)), nodeMap.get(graph.getDest(e)));
		}
		
		return fsm;
	}
	
	public static void addSelfLoops(DirectedGraph<BPPNode, Edge> fsm) {
		for (BPPNode vertex : fsm.getVertices()) {
			vertex.color("lightgrey");
			if (fsm.getPredecessorCount(vertex) != 0) { // Exclude the start state 
				Set<String> props = vertex.getProps();
				Edge selfLoop = new Edge(props);
				fsm.addEdge(selfLoop, vertex, vertex);
			}
		}
	}
	
	public static<V, E> boolean isEndState(DirectedGraph<V, E> fsm, V state) {
		return (fsm.getSuccessorCount(state) == 0);
	}
	
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
	
	public static<V, E> DirectedGraph<V, E> convertToDFA(DirectedGraph<V, E> nfa) {
		// TODO: Implement subset construction
		return null;
	}
}
