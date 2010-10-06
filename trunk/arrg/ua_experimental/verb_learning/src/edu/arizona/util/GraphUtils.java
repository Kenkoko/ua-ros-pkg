package edu.arizona.util;

import java.util.Collection;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;
import java.util.Stack;
import java.util.Vector;

import com.google.common.base.Preconditions;
import com.google.common.collect.HashMultimap;
import com.google.common.collect.SetMultimap;

import edu.arizona.cs.learn.algorithm.markov.BPPNode;
import edu.arizona.cs.learn.util.graph.Edge;
import edu.arizona.planning.fsm.FSMState;
import edu.arizona.planning.fsm.FSMState.StateType;
import edu.arizona.planning.fsm.FSMTransition;
import edu.uci.ics.jung.graph.DirectedGraph;
import edu.uci.ics.jung.graph.DirectedSparseGraph;

public class GraphUtils {
	
//	public static DirectedGraph<FSMState, Edge> convertToFSM(DirectedGraph<BPPNode, Edge> graph) {
//		DirectedGraph<FSMState, Edge> fsm = new DirectedSparseGraph<FSMState, Edge>();
//		
//		addSelfLoops(graph);
//		
//		HashMap<BPPNode, FSMState> nodeMap = new HashMap<BPPNode, FSMState>();
//		for (BPPNode node : graph.getVertices()) {
//			FSMState state = null;
//			if (node.isStart()) {
//				state = new FSMState(StateType.START);
//			} else if (graph.getSuccessorCount(node) == 1) { // TODO: This is because we have already added the self-loops
//				state = new FSMState(StateType.GOOD_TERMINAL);
//			} else {
//				state = new FSMState(StateType.GOOD);
//			}
//			nodeMap.put(node, state);
//		}
//		
//		for (Edge e : graph.getEdges()) {
//			fsm.addEdge(new Edge(e.props()), nodeMap.get(graph.getSource(e)), nodeMap.get(graph.getDest(e)));
//		}
//		
//		return fsm;
//	}
	
//	public static void addSelfLoops(DirectedGraph<BPPNode, Edge> fsm) {
//		for (BPPNode vertex : fsm.getVertices()) {
//			vertex.color("lightgrey");
//			if (fsm.getPredecessorCount(vertex) != 0) { // Exclude the start state 
//				Set<String> props = vertex.getProps();
//				Edge selfLoop = new Edge(props);
//				fsm.addEdge(selfLoop, vertex, vertex);
//			}
//		}
//	}
	
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
	
	public static DirectedGraph<FSMState, FSMTransition> convertToDFA(DirectedGraph<BPPNode, Edge> nfa) {
		BPPNode start = null;
		for (BPPNode vertex : nfa.getVertices()) {
			if (vertex.isStart()) {
				start = vertex;
			}
		}
		HashSet<BPPNode> startSet = new HashSet<BPPNode>();
		startSet.add(start);
		
		// These are the DFA states
		Stack<Set<BPPNode>> openSubsets = new Stack<Set<BPPNode>>(); 
		openSubsets.add(startSet);
		
		Set<Set<BPPNode>> closedSubsets = new HashSet<Set<BPPNode>>();
		
		// These is the DFA "alphabet"
//		Set<Set<String>> alphabet = new HashSet<Set<String>>();
//		for (Edge edge : nfa.getEdges()) {
//			alphabet.add(edge.props());
//		}
		
		Map<Set<BPPNode>, SetMultimap<Set<String>, BPPNode>> transitions = new HashMap<Set<BPPNode>, SetMultimap<Set<String>, BPPNode>>();
		
		while (!openSubsets.isEmpty()) {
			Set<BPPNode> currSubset = openSubsets.pop();
			
			HashMultimap<Set<String>, BPPNode> outEdges = HashMultimap.create();
			for (BPPNode node : currSubset) {
				for (Edge edge : nfa.getOutEdges(node)) {
					outEdges.put(edge.props(), nfa.getDest(edge));
				}
			}
			
			transitions.put(currSubset, outEdges);
			closedSubsets.add(currSubset);
			
			for (Set<String> letter : outEdges.keySet()) {
				Set<BPPNode> newSubset = outEdges.get(letter);
				if (!closedSubsets.contains(newSubset)) {
					openSubsets.add(newSubset);
				}
			}
		}
		
		// Map the subsets to FSMStates
		HashMap<Set<BPPNode>, FSMState> subsetToState = new HashMap<Set<BPPNode>, FSMState>();
		for (Set<BPPNode> subset : closedSubsets) {
			subsetToState.put(subset, new FSMState(StateType.GOOD));
		}
		
		DirectedGraph<FSMState, FSMTransition> dfa = new DirectedSparseGraph<FSMState, FSMTransition>();
		for (FSMState state : subsetToState.values()) {
			System.out.println("ADDING VERTEX: " + state);
			dfa.addVertex(state);
		}
		for (Set<BPPNode> subset : transitions.keySet()) {
			SetMultimap<Set<String>, BPPNode> edgeMap = transitions.get(subset);
			for (Set<String> edgeProps : edgeMap.keySet()) {
				FSMState source = subsetToState.get(subset);
				FSMState dest = subsetToState.get(edgeMap.get(edgeProps));
				FSMTransition edge = new FSMTransition(edgeProps);
				
				System.out.println("ADDING EDGE: " + source + "," + edge + "," + dest);
				
				Preconditions.checkNotNull(source, "source IS NULL");
				Preconditions.checkNotNull(dest, "dest IS NULL");
				Preconditions.checkArgument(!hasEdge(dfa, source, edge, dest), "DUPLICATE EDGE: " + source + " " + edge + " " + dest);
				
				dfa.addEdge(edge, source, dest);
			}
		}
		for (FSMState state : getStartStates(dfa)) {
			state.setType(StateType.START);
		}
		for (FSMState state : getTerminalStates(dfa)) {
			state.setType(StateType.GOOD_TERMINAL);
		}
		
		return dfa;
	}
	
	
	// Generic Utils:
	
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
}
