package edu.arizona.cs.learn.algorithm.markov;

import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.Map;
import java.util.Queue;
import java.util.Set;
import java.util.Stack;

import com.google.common.collect.HashMultimap;
import com.google.common.collect.SetMultimap;

import edu.arizona.cs.learn.util.graph.Edge;
import edu.uci.ics.jung.graph.DirectedGraph;
import edu.uci.ics.jung.graph.DirectedSparseGraph;

public class FSMConverter {
	
	/**
	 * Retrieve the start state of the FSM, where the start node should
	 * be labelled as 'start'.
	 * @param graph - The FSM in question.
	 * @return The start state of the FSM.
	 */
	public static BPPNode getStartState(DirectedGraph<BPPNode, Edge> graph) {
		for (BPPNode n : graph.getVertices()) {
			if (n.label().equals("start")) {
				return n;
			}
		}
		return null;
	}

	/**
	 * Compute the epsilon-closure set for the given state.
	 * NOTE: Since we don't have epsilon transitions in our nfa's,
	 * 		 the eps-closure set of the input just contains the input.
	 * @param node - The node to compute the eps-closure set for.
	 * @return The epsilon-closure set of the given state.
	 */
	private static Set<BPPNode> epsilonClosure(DirectedGraph<BPPNode, Edge> graph, BPPNode node) {
		Set<BPPNode> closureSet = new HashSet<BPPNode>();
		
		closureSet.add(node);
		
		return closureSet;
	}
	
	/**
	 * Collapse the set of 'start' states into a single unique state.
	 * @param startSet - Set of starting states
	 * @return Start state.
	 */
	private static BPPNode collapseStartSet(Set<BPPNode> startSet) {
		return new StringNode("start", null);
	}
	
	/**
	 * Collapse the set of states into a single unique state that's
	 * representative of the set.
	 * 
	 * NOTE: Due to the special property of our NFA, where each state
	 * 		 has a 1:1 relationship with the unique edge that transitions
	 * 		 to it, then the each state in the set of states that can be
	 * 		 reached via some unique set of props must be identical.
	 * 		 Hence, all states in the set must be identical and
	 * 		 collapsing is trivial.
	 * 
	 * @param statesSet - Set of states to collapse.
	 * @param isFinal - Whether or not this set of states contains a final state.
	 * @param startNode - The starting node.
	 * @return A unique node that's representative of the set.
	 */
	private static BPPNode collapseStatesSet(Set<BPPNode> statesSet,
			boolean isFinal, BPPNode startNode) {
		
		Object[] statesArray = statesSet.toArray();
		for (int i = 0; i < (statesArray.length - 1); i++) {
			assert(((BPPNode)statesArray[i]).getProps().containsAll(
					((BPPNode)statesArray[i+1]).getProps()));
		}
		BPPNode n = (BPPNode)statesArray[0];
		
		return new BPPNode(n.getPropList(), new StringBuffer(n.getValues()), startNode, isFinal);
	}
	
	/**
	 * Construct a DFA.
	 * @param startSet - Set of start states.
	 * @param statesSets - State sets.
	 * @param transitions - Transition functions.
	 * @return A DFA.
	 */
	private static DirectedGraph<BPPNode, Edge> constructDFA(
			Set<BPPNode> startSet, Map<Set<BPPNode>, Boolean> statesSets,
			Map<Set<BPPNode>, SetMultimap<Set<String>, BPPNode>> transitions) {
		
		DirectedGraph<BPPNode, Edge> dfa = new DirectedSparseGraph<BPPNode, Edge>();
		Map<Set<BPPNode>, BPPNode> statesTable = new HashMap<Set<BPPNode>, BPPNode>();
		
		// Collapse and add the start state
		BPPNode startNode = collapseStartSet(startSet);
		statesTable.put(startSet, startNode);
		dfa.addVertex(startNode);
		
		// Collapse and add remaining states
		for (Set<BPPNode> set : statesSets.keySet()) {
			BPPNode node = collapseStatesSet(set, statesSets.get(set), startNode);
			statesTable.put(set, node);
			dfa.addVertex(node);
		}
		
		// Add edges
		for (Set<BPPNode> fromSet : transitions.keySet()) {
			BPPNode fromNode = statesTable.get(fromSet);
			SetMultimap<Set<String>, BPPNode> outEdges = transitions.get(fromSet);
			for (Set<String> props : outEdges.keySet()) {
				Set<BPPNode> destSet = outEdges.get(props);
				BPPNode destNode = statesTable.get(destSet);
				Edge e = new Edge(props);
				
				// Sanity checks
				assert(fromNode != null);
				assert(destNode != null);
				assert(dfa.findEdge(fromNode, destNode) == null);
				
				dfa.addEdge(e, fromNode, destNode);
				e.increment();
			}
		}
		
		return dfa;
	}
	
	/**
	 * Convert an NFA to an equivalent DFA.
	 * @param nfa - The NFA.
	 * @return An equivalent DFA.
	 */
	public static DirectedGraph<BPPNode, Edge> convertNFAtoDFA(
			DirectedGraph<BPPNode, Edge> nfa) {
		BPPNode startState = getStartState(nfa);
		return convertNFAtoDFA(nfa, startState);
	}
	
	/**
	 * Convert an NFA to an equivalent DFA.
	 * @param nfa - The NFA.
	 * @param startState - The starting state of the NFA.
	 * @return An equivalent DFA.
	 */
	public static DirectedGraph<BPPNode, Edge> convertNFAtoDFA(
			DirectedGraph<BPPNode, Edge> nfa, BPPNode startState) {
		
		// Initialize new set of DFA states
		Stack<Set<BPPNode>> unmarkedStates = new Stack<Set<BPPNode>>();
		Set<Set<BPPNode>> markedStates = new HashSet<Set<BPPNode>>();
		Map<Set<BPPNode>, SetMultimap<Set<String>, BPPNode>> transitions = 
			new HashMap<Set<BPPNode>, SetMultimap<Set<String>, BPPNode>>();
		
		// Create new DFA start state from the epsilon-closure set of the
		// NFA's start state.
		Set<BPPNode> start = epsilonClosure(nfa, startState);
		unmarkedStates.push(start);

		// While there are still some unmarked states
		while (!unmarkedStates.isEmpty()) {
			
			// Pick an unmarked state as the current state and process it
			Set<BPPNode> state = unmarkedStates.pop();

			// Mark the state as processed
			markedStates.add(state);
						
			// For each transition symbol, find the set of states in the NFA
			// that we can transition to.
			HashMultimap<Set<String>, BPPNode> outEdges = HashMultimap.create();
			for (BPPNode node : state) {
				for (Edge e : nfa.getOutEdges(node)) {
					Set<BPPNode> dest = epsilonClosure(nfa, nfa.getDest(e));
					if (outEdges.containsKey(e.props())) {
						outEdges.get(e.props()).addAll(dest);
					} else {
						outEdges.putAll(e.props(), dest);
					}
				}
			}
			
			// For each transition symbol, if the set of states what we
			// can transition to is not already added to the DFA, then add
			// it as an 'unmarked' state.
			for (Set<String> symbol : outEdges.keySet()) {
				Set<BPPNode> newState = outEdges.get(symbol);
				if (!markedStates.contains(newState)) {
					unmarkedStates.push(newState);
				}
			}
			
			// For each transition symbol, add a transition edge with that
			// symbol from the current state being processed to the set of
			// reachable states.
			transitions.put(state, outEdges);
		}
		
		// Find final states
		markedStates.remove(start);
		Map<Set<BPPNode>, Boolean> statesSets = new HashMap<Set<BPPNode>, Boolean>();
		for (Set<BPPNode> s : markedStates) {
			boolean isFinal = false;
			for (BPPNode n : s) {
				if (nfa.getOutEdges(n).size() == 0) {
					isFinal = true;
					break;
				}
			}
			statesSets.put(s, isFinal);
		}
		DirectedGraph<BPPNode, Edge> dfa = constructDFA(start, statesSets, transitions);
		
		// Dump stats
		dumpStatistics(nfa, dfa);
		
		// Init distance to nearest final for each node
		FSMFactory.initDistanceToNearestFinal(dfa);
		
		return dfa;
	}

	/**
	 * Output basic statistics about the given NFA & DFA.
	 * @param nfa - An NFA.
	 * @param dfa - A DFA.
	 */
	private static void dumpStatistics(DirectedGraph<BPPNode, Edge> nfa,
			DirectedGraph<BPPNode, Edge> dfa) {
		String stats = "-- FSM Conversion Stats --\n" +
			"NFA nodes: " + nfa.getVertexCount() + "\n" +
			"NFA edges: " + nfa.getEdgeCount() + "\n" +
			"DFA nodes: " + dfa.getVertexCount() + "\n" +
			"DFA edges: " + dfa.getEdgeCount() + "\n" +
			"--------------------------\n";
		System.out.println(stats);
	}
	
}
