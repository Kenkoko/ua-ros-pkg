package edu.arizona.util;

import java.util.HashMap;
import java.util.HashSet;
import java.util.Set;
import java.util.Vector;

import edu.arizona.cs.learn.algorithm.markov.BPPNode;
import edu.arizona.cs.learn.util.graph.Edge;
import edu.uci.ics.jung.graph.DirectedGraph;

public class StateMachineUtils {
	
	// TODO: Probably merge with graphutils
	
	public static void convertToDFA(DirectedGraph<BPPNode, Edge> nfa) {
		BPPNode startNode = null;
		
		for (BPPNode n : nfa.getVertices()) {
			if (n.label().equals("start")) {
				startNode = n;
			}
		}
		
		// For now, don't need real NFA to DFA conversion, just "collapsing"
//		Vector<Set<String>> alphabet = getAlphabet(nfa);
//		DirectedGraph<Set<BPPNode>, Edge> preDFA = new DirectedSparseGraph<Set<BPPNode>, Edge>();
//		Set<BPPNode> startSet = new HashSet<BPPNode>();
//		startSet.add(startNode);
//		
//		for (Set<String> letter : alphabet) {
//			Set<BPPNode> newState = new HashSet<BPPNode>();
//			newState.addAll(closure(nfa, startSet, letter));
//		}
		
//		DirectedGraph<BPPNode, Edge> preDFA = new DirectedSparseGraph<BPPNode, Edge>();
		
		Set<BPPNode> frontier = new HashSet<BPPNode>();
		frontier.add(startNode);
		
		while (!frontier.isEmpty()) {
			Set<BPPNode> newFrontier = new HashSet<BPPNode>();
			for (BPPNode node : frontier) {
				HashMap<Set<String>, Set<BPPNode>> propsToSucc = new HashMap<Set<String>, Set<BPPNode>>();
				HashMap<BPPNode, Edge> succToEdge = new HashMap<BPPNode, Edge>();
				Vector<Edge> outEdges = new Vector<Edge>(nfa.getOutEdges(node));
				for (Edge e : outEdges) {
					succToEdge.put(nfa.getDest(e), e);
					if (propsToSucc.containsKey(e.props())) {
						propsToSucc.get(e.props()).add(nfa.getDest(e));
					} else {
						propsToSucc.put(e.props(), new HashSet<BPPNode>());
						propsToSucc.get(e.props()).add(nfa.getDest(e));
					}
				}
				
				for (Set<String> props : propsToSucc.keySet()) {
					Set<BPPNode> succSet = propsToSucc.get(props);
					if (succSet.size() > 1) {
						Vector<BPPNode> succList = new Vector<BPPNode>(succSet);
						BPPNode saved = succList.firstElement();
						Edge savedEdge = succToEdge.get(saved);
						newFrontier.add(saved);
						for (int i = 1; i < succList.size(); i++) {
							savedEdge.increment(); // I think this is what this is for?
							
							BPPNode deadNode = succList.get(i);
							Edge deadEdge = succToEdge.get(deadNode);
							
							for (Edge switched : nfa.getOutEdges(deadNode)) {
								BPPNode target = nfa.getDest(switched);
								nfa.removeEdge(switched);
								nfa.addEdge(switched, saved, target);
							}
							
							nfa.removeEdge(deadEdge);
							nfa.removeVertex(deadNode);
						}
					} else {
						newFrontier.add(succSet.iterator().next());
					}
				}
			}
			
			frontier = newFrontier;
		}

//		return null;
	}
	
	public static Vector<Set<String>> getAlphabet(DirectedGraph<BPPNode, Edge> nfa) {
		Set<Set<String>> alphabet = new HashSet<Set<String>>();
		
		for (Edge e : nfa.getEdges()) {
			alphabet.add(e.props());
		}
		
		return new Vector<Set<String>>(alphabet);
	}
	
	public static Set<BPPNode> closure(DirectedGraph<BPPNode, Edge> nfa, Set<BPPNode> stateSet, Set<String> letter) {
		Set<BPPNode> closureSet = new HashSet<BPPNode>();
		
		for (BPPNode state : stateSet) {
			closureSet.addAll(closure(nfa, state, letter));
		}
		
		return closureSet;
	}
	
	public static Set<BPPNode> closure(DirectedGraph<BPPNode, Edge> nfa, BPPNode state, Set<String> letter) {
		Set<BPPNode> closureSet = new HashSet<BPPNode>();
		
		for (Edge e : nfa.getOutEdges(state)) {
			if (e.props().equals(letter)) {
				closureSet.add(nfa.getDest(e));
			}
		}
		
		return closureSet;
	}
	
//	public static Vector<Vector<Edge>> getChains(DirectedGraph<BPPNode, Edge> graph, BPPNode start) {
//		Vector<Vector<Edge>> chains = new Vector<Vector<Edge>>();
//		
//		for (Edge e : graph.getOutEdges(start)) {
//			Vector<Edge> chain = new Vector<Edge>();
//			chain.add(e);
//			while (!graph.getOutEdges(graph.getDest(chain.lastElement())).isEmpty()) {
//				// There had better be only one! TODO: Throw an exception or something if that's not the case
//				chain.add(graph.getOutEdges(graph.getDest(chain.lastElement())).iterator().next());
//			}
//			chains.add(chain);
//		}
//		
//		return chains;
//	}
	
//	Vector<Vector<Edge>> chains = getChains(nfa, startNode);
//	for (Vector<Edge> chain : chains) {
//		if (dfa.getEdgeCount() == 0) { // Just add the first chain automatically
//			for (Edge e : chain) {
//				dfa.addEdge(e, nfa.getSource(e), nfa.getDest(e));
//			}
//		} else {
//			BPPNode currNode = startNode;
//			boolean ok = true;
//			for (Edge e : chain) {
//				BPPNode source = nfa.getSource(e);
//				BPPNode dest = nfa.getDest(e);
//				
//				if (source.label().equals(currNode.label())) {
//					
//				} else {
//					ok = false;
//				}
//			}
//		}
//	}
}
