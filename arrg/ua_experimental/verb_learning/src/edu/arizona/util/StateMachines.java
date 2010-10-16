package edu.arizona.util;

import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;
import java.util.Stack;

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

public class StateMachines {
	
	/***
	 * Convert NFA to DFA 
	 */
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
//			System.out.println("ADDING VERTEX: " + state);
			dfa.addVertex(state);
		}
		for (Set<BPPNode> subset : transitions.keySet()) {
			SetMultimap<Set<String>, BPPNode> edgeMap = transitions.get(subset);
			for (Set<String> edgeProps : edgeMap.keySet()) {
				FSMState source = subsetToState.get(subset);
				FSMState dest = subsetToState.get(edgeMap.get(edgeProps));
				FSMTransition edge = new FSMTransition(edgeProps);
				
//				System.out.println("ADDING EDGE: " + source + "," + edge + "," + dest);
				
				Preconditions.checkNotNull(source, "source IS NULL");
				Preconditions.checkNotNull(dest, "dest IS NULL");
				Preconditions.checkArgument(!Graphs.hasEdge(dfa, source, edge, dest), "DUPLICATE EDGE: " + source + " " + edge + " " + dest);
				
				dfa.addEdge(edge, source, dest);
			}
		}
		for (FSMState state : Graphs.getStartStates(dfa)) {
			state.setType(StateType.START);
		}
		for (FSMState state : Graphs.getTerminalStates(dfa)) {
			state.setType(StateType.GOOD_TERMINAL);
		}
		
		return dfa;
	}
	
	
	
	
//	private void minimizeFast(DirectedGraph<FSMState, FSMTransition> dfa) {
//		DirectedGraph<FSMState, FSMTransition> newDFA = new DirectedSparseGraph<FSMState, FSMTransition>();
//		
//		Set<Set<FSMState>> pi = new HashSet<Set<FSMState>>();
//		// First initial partition is all accepting states
//		pi.add(new HashSet<FSMState>(Collections2.filter(dfa.getVertices(), FSMState.isTerminal)));
//		// Second initial partition is all non-accepting states
//		pi.add(new HashSet<FSMState>(Collections2.filter(dfa.getVertices(), Predicates.not(FSMState.isTerminal))));
//		
//		Set<Set<FSMState>> piNew = newPartition(pi);
//		while (!piNew.equals(pi)) {
//			pi = piNew;
//			piNew = newPartition(pi);
//		}
//		
//		Set<Set<FSMState>> piFinal = pi;
//		
//		// TODO: Still need to merge states
//	}
//	
//	private Set<Set<FSMState>> newPartition(Set<Set<FSMState>> pi) {
//		Set<Set<FSMState>> result = new HashSet<Set<FSMState>>();
//		
//		for (Set<FSMState> s : pi) {
//			Set<FSMState> newSet = new HashSet<FSMState>();
//			
//			for (FSMState state : s) {
//				for (FSMTransition t : dfa_.getOutEdges(state)) {
//					final FSMState opposite = dfa_.getOpposite(state, t);
//					Set<FSMState> destSet = CollectionUtils.find(pi, new Predicate<Set<FSMState>>() {
//						public boolean value(Set<FSMState> subset) {
//							return subset.contains(opposite);
//						}
//					});
//				}
//			}
//		}
//		
//		return result;
//	}
	
	
	
	
}
