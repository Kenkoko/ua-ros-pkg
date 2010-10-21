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
	
	public static DirectedGraph<FSMState, FSMTransition> combineDFAs(DirectedGraph<FSMState, FSMTransition> pos, DirectedGraph<FSMState, FSMTransition> neg) {
		for (FSMState vertex : neg.getVertices()) {
			pos.addVertex(vertex);
		}
		for (FSMTransition transition : neg.getEdges()) {
			pos.addEdge(transition, neg.getEndpoints(transition));
		}
		
		return subsetConstruction(pos);
	}
	
	public static StateType getSubsetType(Set<FSMState> subset) {
		StateType type = StateType.GOOD;
		
		System.out.println("BEGIN SUBSET");
		
		for (FSMState state : subset) {
			switch(state.getType()) {
			case BAD_TERMINAL:
				type = StateType.BAD_TERMINAL;
				break;
			case GOOD_TERMINAL:
				type = StateType.GOOD_TERMINAL;
				break;
			case GOOD:
				break;
			case START:
				type = StateType.START;
				break;
			}
			
			System.out.println("TYPE CHECK: " + type); // Need to make sure there are no mixed subsets
		}
		
		return type;
	}
	
	public static DirectedGraph<FSMState, FSMTransition> subsetConstruction(DirectedGraph<FSMState, FSMTransition> nfa) {
		HashSet<FSMState> startSet = new HashSet<FSMState>();
		for (FSMState vertex : nfa.getVertices()) {
			if (vertex.getType().equals(StateType.START)) {
				startSet.add(vertex);
			}
		}
		
		// These are the DFA states
		Stack<Set<FSMState>> openSubsets = new Stack<Set<FSMState>>(); 
		openSubsets.add(startSet);
		
		Set<Set<FSMState>> closedSubsets = new HashSet<Set<FSMState>>();
		
		Map<Set<FSMState>, SetMultimap<Set<String>, FSMState>> transitions = new HashMap<Set<FSMState>, SetMultimap<Set<String>, FSMState>>();
		
		while (!openSubsets.isEmpty()) {
			Set<FSMState> currSubset = openSubsets.pop();
			
			HashMultimap<Set<String>, FSMState> outEdges = HashMultimap.create();
			for (FSMState node : currSubset) {
				for (FSMTransition edge : nfa.getOutEdges(node)) {
					outEdges.put(edge.getSymbol(), nfa.getDest(edge));
				}
			}
			
			transitions.put(currSubset, outEdges);
			closedSubsets.add(currSubset);
			
			for (Set<String> letter : outEdges.keySet()) {
				Set<FSMState> newSubset = outEdges.get(letter);
				if (!closedSubsets.contains(newSubset)) {
					openSubsets.add(newSubset);
				}
			}
		}
		
		// Map the subsets to FSMStates
		HashMap<Set<FSMState>, FSMState> subsetToState = new HashMap<Set<FSMState>, FSMState>();
		for (Set<FSMState> subset : closedSubsets) {
			subsetToState.put(subset, new FSMState(getSubsetType(subset)));
		}
		
		DirectedGraph<FSMState, FSMTransition> dfa = new DirectedSparseGraph<FSMState, FSMTransition>();
		for (FSMState state : subsetToState.values()) {
			dfa.addVertex(state);
		}
		for (Set<FSMState> subset : transitions.keySet()) {
			SetMultimap<Set<String>, FSMState> edgeMap = transitions.get(subset);
			for (Set<String> edgeProps : edgeMap.keySet()) {
				FSMState source = subsetToState.get(subset);
				FSMState dest = subsetToState.get(edgeMap.get(edgeProps));
				FSMTransition edge = new FSMTransition(edgeProps);
				
				Preconditions.checkNotNull(source, "source IS NULL");
				Preconditions.checkNotNull(dest, "dest IS NULL");
				Preconditions.checkArgument(!Graphs.hasEdge(dfa, source, edge, dest), "DUPLICATE EDGE: " + source + " " + edge + " " + dest);
				
				dfa.addEdge(edge, source, dest);
			}
		}
		
		return dfa;
	}
	
	
	
	/***
	 * Convert NFA to DFA 
	 */
	public static<N,E> DirectedGraph<FSMState, FSMTransition> convertToDFA(DirectedGraph<N, E> nfa, boolean isGood) {
		HashSet<N> startSet = new HashSet<N>();
		for (N vertex : nfa.getVertices()) {
			// SUPER HACKY TIME
			if (vertex instanceof BPPNode && ((BPPNode) vertex).isStart()) {
				startSet.add(vertex);
			} else if (vertex instanceof FSMState && ((FSMState) vertex).getType().equals(StateType.START)) {
				startSet.add(vertex);
			}
		}
		
		// These are the DFA states
		Stack<Set<N>> openSubsets = new Stack<Set<N>>(); 
		openSubsets.add(startSet);
		
		Set<Set<N>> closedSubsets = new HashSet<Set<N>>();
		
		Map<Set<N>, SetMultimap<Set<String>, N>> transitions = new HashMap<Set<N>, SetMultimap<Set<String>, N>>();
		
		while (!openSubsets.isEmpty()) {
			Set<N> currSubset = openSubsets.pop();
			
			HashMultimap<Set<String>, N> outEdges = HashMultimap.create();
			for (N node : currSubset) {
				for (E edge : nfa.getOutEdges(node)) {
					// ONE MORE HACK
					if (edge instanceof Edge) {
						outEdges.put(((Edge) edge).props(), nfa.getDest(edge));
					} else if (edge instanceof FSMTransition) {
						outEdges.put(((FSMTransition) edge).getSymbol(), nfa.getDest(edge));
					}
				}
			}
			
			transitions.put(currSubset, outEdges);
			closedSubsets.add(currSubset);
			
			for (Set<String> letter : outEdges.keySet()) {
				Set<N> newSubset = outEdges.get(letter);
				if (!closedSubsets.contains(newSubset)) {
					openSubsets.add(newSubset);
				}
			}
		}
		
		// Map the subsets to FSMStates
		HashMap<Set<N>, FSMState> subsetToState = new HashMap<Set<N>, FSMState>();
		for (Set<N> subset : closedSubsets) {
			subsetToState.put(subset, new FSMState(StateType.GOOD));
		}
		
		DirectedGraph<FSMState, FSMTransition> dfa = new DirectedSparseGraph<FSMState, FSMTransition>();
		for (FSMState state : subsetToState.values()) {
//			System.out.println("ADDING VERTEX: " + state);
			dfa.addVertex(state);
		}
		for (Set<N> subset : transitions.keySet()) {
			SetMultimap<Set<String>, N> edgeMap = transitions.get(subset);
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
			if (isGood) {
				state.setType(StateType.GOOD_TERMINAL);
			} else {
				state.setType(StateType.BAD_TERMINAL);
			}
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
