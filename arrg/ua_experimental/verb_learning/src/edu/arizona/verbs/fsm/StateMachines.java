package edu.arizona.verbs.fsm;

import java.util.Collection;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;
import java.util.Stack;
import java.util.TreeSet;
import java.util.Vector;

import com.google.common.base.Function;
import com.google.common.base.Preconditions;
import com.google.common.collect.Collections2;
import com.google.common.collect.HashMultimap;
import com.google.common.collect.Maps;
import com.google.common.collect.SetMultimap;
import com.google.common.collect.Sets;
import com.google.common.collect.Sets.SetView;

import edu.arizona.cs.learn.algorithm.markov.BPPNode;
import edu.arizona.cs.learn.util.graph.Edge;
import edu.arizona.verbs.fsm.FSMState.StateType;
import edu.arizona.verbs.util.Graphs;
import edu.arizona.verbs.util.Predicate;
import edu.uci.ics.jung.graph.DirectedGraph;
import edu.uci.ics.jung.graph.DirectedSparseGraph;
import edu.uci.ics.jung.graph.util.Pair;

public class StateMachines {
	
	public static TreeSet<FSMState> getStartState(DirectedGraph<FSMState, FSMTransition> fsm) {
		Collection<FSMState> startStates = 
			Collections2.filter(fsm.getVertices(), new Predicate<FSMState>() {
			public boolean value(FSMState arg) {
				return arg.getType().equals(StateType.START);
			}});
		return new TreeSet<FSMState>(startStates);
	}
	
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
	
	public static DirectedGraph<FSMState, FSMTransition> duplicate(DirectedGraph<FSMState, FSMTransition> fsm) {
		DirectedGraph<FSMState, FSMTransition> duplicate = new DirectedSparseGraph<FSMState, FSMTransition>();
	
		// Add all edges from this FSM
		HashMap<FSMState, FSMState> stateMap1 = Maps.newHashMap();
		for (FSMTransition ot : fsm.getEdges()) {
			FSMState os = fsm.getSource(ot);
			if (!stateMap1.containsKey(os)) {
				stateMap1.put(os, new FSMState(os.getType()));
			}
		
			FSMState od = fsm.getDest(ot);
			if (!stateMap1.containsKey(od)) {
				stateMap1.put(od, new FSMState(od.getType()));
			}
		
			duplicate.addEdge(new FSMTransition(ot.getActiveRelations()), stateMap1.get(os), stateMap1.get(od));
		}
	
		return duplicate;
	}
	
	public static boolean hasTransition(DirectedGraph<FSMState, FSMTransition> dfa, FSMState source, Set<String> symbol, FSMState dest) {
		Collection<FSMTransition> set = matchTransition(dfa, source, symbol);
		if (set.isEmpty()) {
			return false;
		} else {
			for (FSMTransition t : set) {
				if (dfa.getOpposite(source, t).equals(dest)){
					return true;
				}
			}
			return false;
		}
	}
	
	public static Collection<FSMTransition> matchTransition(DirectedGraph<FSMState, FSMTransition> dfa, FSMState state, final Set<String> symbol) {
		Collection<FSMTransition> matchingTransitions = Collections2.filter(dfa.getOutEdges(state), new Predicate<FSMTransition>() {
			public boolean value(FSMTransition arg) { return arg.getSymbol().equals(symbol); }});
		
		return matchingTransitions;
	}
	
	/***
	 * Minimize DFA using simpler O(n^2) algorithm. 
	 */
	public static void minimizeSlow(DirectedGraph<FSMState, FSMTransition> dfa) {
		System.out.println("========= BEGIN DFA MINIMIZATION");
		
		Vector<FSMState> states = new Vector<FSMState>(dfa.getVertices());
		
		Set<Pair<FSMState>> allPairs = new HashSet<Pair<FSMState>>(); 
		Set<Pair<FSMState>> distinct = new HashSet<Pair<FSMState>>();
		Set<Pair<FSMState>> oldDistinct = new HashSet<Pair<FSMState>>();
		for (int i = 0; i < states.size(); i++) {
			for (int j = i+1; j < states.size(); j++) {
				FSMState p = states.get(i);
				FSMState q = states.get(j);
				Pair<FSMState> pair = new Pair<FSMState>(p, q);
				allPairs.add(pair);
				if (p.isTerminal() != q.isTerminal()) { // If one is accepting and the other is not, must be distinct
					distinct.add(pair);
				} else if (p.isTerminal() && q.isTerminal()) {
					if (!p.getType().equals(q.getType())) {
						distinct.add(pair);
					}
				}
			}
		}
		
		Function<FSMTransition, Set<String>> getSymbol = new Function<FSMTransition, Set<String>>() {
			public Set<String> apply(FSMTransition t) {return t.getSymbol();} };
		
		while (!distinct.equals(oldDistinct)) {
			oldDistinct = distinct;
			for (Pair<FSMState> pair : Sets.difference(allPairs, distinct)) {
				if (!distinct.contains(pair)) {
					FSMState p = pair.getFirst();
					FSMState q = pair.getSecond();
					
					Set<FSMTransition> pEdges = Sets.newHashSet(dfa.getOutEdges(p));
					Set<FSMTransition> qEdges = Sets.newHashSet(dfa.getOutEdges(q));
					Set<Set<String>> pSymbols = Sets.newHashSet(Collections2.transform(pEdges, getSymbol));
					Set<Set<String>> qSymbols = Sets.newHashSet(Collections2.transform(qEdges, getSymbol));
					if (pSymbols.equals(qSymbols)) {
						for (FSMTransition t : pEdges) {
							FSMState pPrime = dfa.getOpposite(p, t);
							Collection<FSMTransition> matches = StateMachines.matchTransition(dfa, q, t.getSymbol());
							if (matches.size() != 1) { throw new RuntimeException("NOT A DFA"); }
							FSMState qPrime = dfa.getOpposite(q, matches.iterator().next());
							Pair<FSMState> newPair;
							if (states.indexOf(pPrime) < states.indexOf(qPrime)) {
								newPair = new Pair<FSMState>(pPrime, qPrime);
							} else {
								newPair = new Pair<FSMState>(qPrime, pPrime);
							}
							if (distinct.contains(newPair)) {
								distinct.add(pair);
							}
						}
					} else { // This is true since we only specify some of the transitions
						distinct.add(pair);
					}
				}
			}
		}
		
		// Now that we know which are distinguishable, we need to delete the unnecessary states
		SetView<Pair<FSMState>> equivalentPairs = Sets.difference(allPairs, distinct);
		Set<Set<FSMState>> subsets = new HashSet<Set<FSMState>>();
		
		for (Pair<FSMState> same : equivalentPairs) {
			Set<Set<FSMState>> toMerge = new HashSet<Set<FSMState>>();
			for (Set<FSMState> set : subsets) {
				if (set.contains(same.getFirst()) || set.contains(same.getSecond())) {
					toMerge.add(set);
				}
			}
			
			Set<FSMState> newSubset = new HashSet<FSMState>();
			newSubset.add(same.getFirst());
			newSubset.add(same.getSecond());
			for (Set<FSMState> oldSubset : toMerge) {
				newSubset.addAll(oldSubset);
				subsets.remove(oldSubset);
			}
			subsets.add(newSubset);
		}
		
		for (Set<FSMState> subset : subsets) {
			System.out.println(subset);
			
			FSMState representative = subset.iterator().next();
			subset.remove(representative);
			
			for (FSMState dead : subset) {
				System.out.println("deleting state: " + dead + ", merging into: " + representative);
				for (FSMTransition t : dfa.getInEdges(dead)) {
					FSMState p = dfa.getOpposite(dead, t);
					safeAddTransition(dfa, p, t.getSymbol(), representative);
				}
				for (FSMTransition t : dfa.getOutEdges(dead)) {
					FSMState p = dfa.getOpposite(dead, t);
					safeAddTransition(dfa, representative, t.getSymbol(), p);
				}
				
				// It's all over for him
				for (FSMTransition t : dfa.getIncidentEdges(dead)) {
					dfa.removeEdge(t);
				}
				dfa.removeVertex(dead);
			}
		}
		
		System.out.println("========= END DFA MINIMIZATION");
	}
	
	public static void safeAddTransition(DirectedGraph<FSMState, FSMTransition> dfa, FSMState source, Set<String> symbol, FSMState dest) {
		if (!StateMachines.hasTransition(dfa, source, symbol, dest)) {
			dfa.addEdge(new FSMTransition(symbol), source, dest);
		} 
	}
	
}
