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
import edu.arizona.verbs.fsm.FSMNode.StateType;
import edu.arizona.verbs.fsm.core.CorePath;
import edu.arizona.verbs.fsm.core.RelationalState;
import edu.arizona.verbs.util.Graphs;
import edu.arizona.verbs.util.Predicate;
import edu.uci.ics.jung.graph.DirectedGraph;
import edu.uci.ics.jung.graph.DirectedSparseGraph;
import edu.uci.ics.jung.graph.util.Pair;

public class StateMachines {
	
	public static TreeSet<FSMNode> getStartState(DirectedGraph<FSMNode, FSMTransition> fsm) {
		Collection<FSMNode> startStates = 
			Collections2.filter(fsm.getVertices(), new Predicate<FSMNode>() {
			public boolean value(FSMNode arg) {
				return arg.getType().equals(StateType.START);
			}});
		return new TreeSet<FSMNode>(startStates);
	}
	
	public static DirectedGraph<FSMNode, FSMTransition> combineDFAs(DirectedGraph<FSMNode, FSMTransition> pos, DirectedGraph<FSMNode, FSMTransition> neg) {
		for (FSMNode vertex : neg.getVertices()) {
			pos.addVertex(vertex);
		}
		for (FSMTransition transition : neg.getEdges()) {
			pos.addEdge(transition, neg.getEndpoints(transition));
		}
		
		return subsetConstruction(pos);
	}
	
	public static StateType getSubsetType(Set<FSMNode> subset) {
		StateType type = StateType.START;
		
		System.out.println("BEGIN SUBSET");
		
		for (FSMNode state : subset) {
			switch(state.getType()) {
			case TERMINAL:
				return StateType.TERMINAL;
			case INTERIOR:
				type = StateType.START;
				break;
			case START:
				break;
			}
			
			System.out.println("TYPE CHECK: " + type); // Need to make sure there are no mixed subsets
		}
		
		return type;
	}
	
	public static DirectedGraph<FSMNode, FSMTransition> subsetConstruction(DirectedGraph<FSMNode, FSMTransition> nfa) {
		HashSet<FSMNode> startSet = new HashSet<FSMNode>();
		for (FSMNode vertex : nfa.getVertices()) {
			if (vertex.getType().equals(StateType.START)) {
				startSet.add(vertex);
			}
		}
		
		// These are the DFA states
		Stack<Set<FSMNode>> openSubsets = new Stack<Set<FSMNode>>(); 
		openSubsets.add(startSet);
		
		Set<Set<FSMNode>> closedSubsets = new HashSet<Set<FSMNode>>();
		
		Map<Set<FSMNode>, SetMultimap<Set<String>, FSMNode>> transitions = new HashMap<Set<FSMNode>, SetMultimap<Set<String>, FSMNode>>();
		
		while (!openSubsets.isEmpty()) {
			Set<FSMNode> currSubset = openSubsets.pop();
			
			HashMultimap<Set<String>, FSMNode> outEdges = HashMultimap.create();
			for (FSMNode node : currSubset) {
				for (FSMTransition edge : nfa.getOutEdges(node)) {
					outEdges.put(edge.getSymbol(), nfa.getDest(edge));
				}
			}
			
			transitions.put(currSubset, outEdges);
			closedSubsets.add(currSubset);
			
			for (Set<String> letter : outEdges.keySet()) {
				Set<FSMNode> newSubset = outEdges.get(letter);
				if (!closedSubsets.contains(newSubset)) {
					openSubsets.add(newSubset);
				}
			}
		}
		
		// Map the subsets to FSMStates
		HashMap<Set<FSMNode>, FSMNode> subsetToState = new HashMap<Set<FSMNode>, FSMNode>();
		for (Set<FSMNode> subset : closedSubsets) {
			subsetToState.put(subset, new FSMNode(getSubsetType(subset)));
		}
		
		DirectedGraph<FSMNode, FSMTransition> dfa = new DirectedSparseGraph<FSMNode, FSMTransition>();
		for (FSMNode state : subsetToState.values()) {
			dfa.addVertex(state);
		}
		for (Set<FSMNode> subset : transitions.keySet()) {
			SetMultimap<Set<String>, FSMNode> edgeMap = transitions.get(subset);
			for (Set<String> edgeProps : edgeMap.keySet()) {
				FSMNode source = subsetToState.get(subset);
				FSMNode dest = subsetToState.get(edgeMap.get(edgeProps));
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
	public static<N,E> DirectedGraph<FSMNode, FSMTransition> convertToDFA(DirectedGraph<N, E> nfa) {
		HashSet<N> startSet = new HashSet<N>();
		for (N vertex : nfa.getVertices()) {
			// SUPER HACKY TIME
			if (vertex instanceof BPPNode && ((BPPNode) vertex).isStart()) {
				startSet.add(vertex);
			} else if (vertex instanceof FSMNode && ((FSMNode) vertex).getType().equals(StateType.START)) {
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
		HashMap<Set<N>, FSMNode> subsetToState = new HashMap<Set<N>, FSMNode>();
		for (Set<N> subset : closedSubsets) {
			subsetToState.put(subset, new FSMNode(StateType.INTERIOR));
		}
		
		DirectedGraph<FSMNode, FSMTransition> dfa = new DirectedSparseGraph<FSMNode, FSMTransition>();
		for (FSMNode state : subsetToState.values()) {
//			System.out.println("ADDING VERTEX: " + state);
			dfa.addVertex(state);
		}
		for (Set<N> subset : transitions.keySet()) {
			SetMultimap<Set<String>, N> edgeMap = transitions.get(subset);
			for (Set<String> edgeProps : edgeMap.keySet()) {
				FSMNode source = subsetToState.get(subset);
				FSMNode dest = subsetToState.get(edgeMap.get(edgeProps));
				FSMTransition edge = new FSMTransition(edgeProps);
				
//				System.out.println("ADDING EDGE: " + source + "," + edge + "," + dest);
				
				Preconditions.checkNotNull(source, "source IS NULL");
				Preconditions.checkNotNull(dest, "dest IS NULL");
				Preconditions.checkArgument(!Graphs.hasEdge(dfa, source, edge, dest), "DUPLICATE EDGE: " + source + " " + edge + " " + dest);
				
				dfa.addEdge(edge, source, dest);
			}
		}
		for (FSMNode state : Graphs.getStartStates(dfa)) {
			state.setType(StateType.START);
		}
		for (FSMNode state : Graphs.getTerminalStates(dfa)) {
			state.setType(StateType.TERMINAL);
		}
		
		return dfa;
	}
	
	public static DirectedGraph<FSMNode, FSMTransition> duplicate(DirectedGraph<FSMNode, FSMTransition> fsm) {
		DirectedGraph<FSMNode, FSMTransition> duplicate = new DirectedSparseGraph<FSMNode, FSMTransition>();
	
		// Add all edges from this FSM
		HashMap<FSMNode, FSMNode> stateMap1 = Maps.newHashMap();
		for (FSMTransition ot : fsm.getEdges()) {
			FSMNode os = fsm.getSource(ot);
			if (!stateMap1.containsKey(os)) {
				stateMap1.put(os, new FSMNode(os.getType()));
			}
		
			FSMNode od = fsm.getDest(ot);
			if (!stateMap1.containsKey(od)) {
				stateMap1.put(od, new FSMNode(od.getType()));
			}
		
			duplicate.addEdge(new FSMTransition(ot.getActiveRelations()), stateMap1.get(os), stateMap1.get(od));
		}
	
		return duplicate;
	}
	
	public static boolean hasTransition(DirectedGraph<FSMNode, FSMTransition> dfa, FSMNode source, Set<String> symbol, FSMNode dest) {
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
	
	public static Collection<FSMTransition> matchTransition(DirectedGraph<FSMNode, FSMTransition> dfa, FSMNode state, final Set<String> symbol) {
		Collection<FSMTransition> matchingTransitions = Collections2.filter(dfa.getOutEdges(state), new Predicate<FSMTransition>() {
			public boolean value(FSMTransition arg) { return arg.getSymbol().equals(symbol); }});
		
		return matchingTransitions;
	}
	
	/***
	 * Minimize DFA using simpler O(n^2) algorithm. 
	 */
	public static void minimizeSlow(DirectedGraph<FSMNode, FSMTransition> dfa) {
		System.out.println("========= BEGIN DFA MINIMIZATION");
		
		Vector<FSMNode> states = new Vector<FSMNode>(dfa.getVertices());
		
		Set<Pair<FSMNode>> allPairs = new HashSet<Pair<FSMNode>>(); 
		Set<Pair<FSMNode>> distinct = new HashSet<Pair<FSMNode>>();
		Set<Pair<FSMNode>> oldDistinct = new HashSet<Pair<FSMNode>>();
		for (int i = 0; i < states.size(); i++) {
			for (int j = i+1; j < states.size(); j++) {
				FSMNode p = states.get(i);
				FSMNode q = states.get(j);
				Pair<FSMNode> pair = new Pair<FSMNode>(p, q);
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
			for (Pair<FSMNode> pair : Sets.difference(allPairs, distinct)) {
				if (!distinct.contains(pair)) {
					FSMNode p = pair.getFirst();
					FSMNode q = pair.getSecond();
					
					Set<FSMTransition> pEdges = Sets.newHashSet(dfa.getOutEdges(p));
					Set<FSMTransition> qEdges = Sets.newHashSet(dfa.getOutEdges(q));
					Set<Set<String>> pSymbols = Sets.newHashSet(Collections2.transform(pEdges, getSymbol));
					Set<Set<String>> qSymbols = Sets.newHashSet(Collections2.transform(qEdges, getSymbol));
					if (pSymbols.equals(qSymbols)) {
						for (FSMTransition t : pEdges) {
							FSMNode pPrime = dfa.getOpposite(p, t);
							Collection<FSMTransition> matches = StateMachines.matchTransition(dfa, q, t.getSymbol());
							if (matches.size() != 1) { throw new RuntimeException("NOT A DFA"); }
							FSMNode qPrime = dfa.getOpposite(q, matches.iterator().next());
							Pair<FSMNode> newPair;
							if (states.indexOf(pPrime) < states.indexOf(qPrime)) {
								newPair = new Pair<FSMNode>(pPrime, qPrime);
							} else {
								newPair = new Pair<FSMNode>(qPrime, pPrime);
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
		SetView<Pair<FSMNode>> equivalentPairs = Sets.difference(allPairs, distinct);
		Set<Set<FSMNode>> subsets = new HashSet<Set<FSMNode>>();
		
		for (Pair<FSMNode> same : equivalentPairs) {
			Set<Set<FSMNode>> toMerge = new HashSet<Set<FSMNode>>();
			for (Set<FSMNode> set : subsets) {
				if (set.contains(same.getFirst()) || set.contains(same.getSecond())) {
					toMerge.add(set);
				}
			}
			
			Set<FSMNode> newSubset = new HashSet<FSMNode>();
			newSubset.add(same.getFirst());
			newSubset.add(same.getSecond());
			for (Set<FSMNode> oldSubset : toMerge) {
				newSubset.addAll(oldSubset);
				subsets.remove(oldSubset);
			}
			subsets.add(newSubset);
		}
		
		for (Set<FSMNode> subset : subsets) {
			System.out.println(subset);
			
			FSMNode representative = subset.iterator().next();
			subset.remove(representative);
			
			for (FSMNode dead : subset) {
				System.out.println("deleting state: " + dead + ", merging into: " + representative);
				for (FSMTransition t : dfa.getInEdges(dead)) {
					FSMNode p = dfa.getOpposite(dead, t);
					safeAddTransition(dfa, p, t.getSymbol(), representative);
				}
				for (FSMTransition t : dfa.getOutEdges(dead)) {
					FSMNode p = dfa.getOpposite(dead, t);
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
	
	public static void safeAddTransition(DirectedGraph<FSMNode, FSMTransition> dfa, FSMNode source, Set<String> symbol, FSMNode dest) {
		if (!StateMachines.hasTransition(dfa, source, symbol, dest)) {
			dfa.addEdge(new FSMTransition(symbol), source, dest);
		} 
	}
	
	/* NEW STUFF */
	
	// Actually, we don't even need good or bad terminals anymore, just terminals
	public static DirectedGraph<FSMNode, FSMTransition> createFSM(Set<CorePath> corePaths) {
		DirectedSparseGraph<FSMNode, FSMTransition> nfa = new DirectedSparseGraph<FSMNode, FSMTransition>();

		// Add the start state
		FSMNode start = new FSMNode(StateType.START);
		nfa.addVertex(start);
		
		if (corePaths.isEmpty()) { // If there are no core paths, the FSM is just a single start state, so we're done
			return nfa;
		}
		
		for (CorePath path : corePaths) {
			Vector<RelationalState> states = path.getPath();
			
			HashMap<RelationalState, FSMNode> stateMap = new HashMap<RelationalState, FSMNode>();
			stateMap.put(states.lastElement(), new FSMNode(StateType.TERMINAL)); // The last is always a terminal
			for (RelationalState state : states) {
				if (!stateMap.containsKey(state)) {
					stateMap.put(state, new FSMNode(StateType.INTERIOR));
				}
			}
			
			nfa.addEdge(new FSMTransition(states.firstElement().getRelations()), start, stateMap.get(states.firstElement()));
			for (int i = 0; i < states.size() - 1; i++) {
				RelationalState curr = states.get(i);
				RelationalState next = states.get(i+1);
				nfa.addEdge(new FSMTransition(next.getRelations()), stateMap.get(curr), stateMap.get(next));
			}
		}
		
		// At this point, we have an NFA
		// Now we convert to DFA (structurally, at least)...
		DirectedGraph<FSMNode, FSMTransition> dfa = convertToDFA(nfa);
		// And minimize
		
		minimizeSlow(dfa); // TODO Is there a problem here? If so, why?
		
		return dfa;
	}
	
	// TODO Also need to validate that FSM is a DAG somewhere
	
	// This is lame but the point is to see if the Jung version is buggy
	public static int findMinDistToTerminal(DirectedGraph<FSMNode, FSMTransition> fsm, FSMNode s) {
		
		if (s.isTerminal()) {
			return 0;
		
		} else if (fsm.getOutEdges(s).isEmpty()) {
			if (s.getType().equals(StateType.START)) {
				return Integer.MAX_VALUE; // return the max value
			} else {
				throw new RuntimeException("FSM IS MALFORMED: INTERIOR STATE HAS NO OUTGOING EDGES");
			}
			
		} else {
			int minDist = Integer.MAX_VALUE;
			
			for (FSMTransition t : fsm.getOutEdges(s)) {
				FSMNode dest = fsm.getDest(t);
				
				if (!dest.equals(s)) { // Break self-loops
					int dist = 1 + findMinDistToTerminal(fsm, dest);
					minDist = Math.min(dist, minDist);
				}
			}
			
			return minDist;
		}
	}
}
