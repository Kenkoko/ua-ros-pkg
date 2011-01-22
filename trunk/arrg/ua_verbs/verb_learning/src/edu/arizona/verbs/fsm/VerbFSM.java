package edu.arizona.verbs.fsm;

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.Vector;

import org.apache.log4j.Logger;

import com.google.common.collect.Collections2;
import com.google.common.collect.Iterables;
import com.google.common.collect.Maps;

import edu.arizona.verbs.fsm.FSMNode.StateType;
import edu.arizona.verbs.fsm.core.CorePath;
import edu.arizona.verbs.shared.Remappable;
import edu.uci.ics.jung.algorithms.shortestpath.UnweightedShortestPath;
import edu.uci.ics.jung.graph.DirectedGraph;
import edu.uci.ics.jung.graph.DirectedSparseGraph;

public class VerbFSM implements Remappable<VerbFSM> {
	public static Logger logger = Logger.getLogger(VerbFSM.class);
	
	public class TransitionResult {
		public FSMState newState = new FSMState();
		public int cost = 0;
		public double heuristic = 0.0;
	}
	
	private DirectedGraph<FSMNode, FSMTransition> dfa_;
	private FSMState startState_ = new FSMState();
	private FSMState activeState_ = new FSMState();
	private Set<FSMNode> terminalNodes_;
	private int startDist_;
	
	private VerbFSM(DirectedGraph<FSMNode, FSMTransition> dfa) {
		dfa_ = dfa;
		init();
	}
	
	public VerbFSM(Set<CorePath> corePaths) {
		dfa_ = StateMachines.createFSM(corePaths);
		init();
	}

	private void init() {
		populateTerminals(); 
		populateStartState();
		
		activeState_ = startState_;
		printStateValues();
	}
	
	public double transitionDFA(Set<String> activeProps) {
		TransitionResult result = simulateDfaTransition(activeState_, activeProps);
		activeState_ = result.newState;
		return result.cost;
	}

	public double transitionNFA(Set<String> activeProps) {
		TransitionResult result = simulateNfaTransition(activeState_, activeProps);
		activeState_ = result.newState;
		return result.cost;
	}
	
	public FSMState getStartState() {
		return startState_;
	}
	
	public int getMinDistToTerminal(FSMState newState) {
		switch (newState.getStateType()) {
		case TERMINAL: // You're already there
			return 0; 
		case START: // Start state cost is already computed
			return startDist_;
		case INTERIOR: // Retrieve the cached value
			return computeMinDistToTerminal(newState);
		default:
			throw new RuntimeException("IMPOSSIBLE");
		}
	}
	
	public int getMinDistToTerminal(FSMNode newState) {
		switch (newState.getType()) {
		case TERMINAL: // You're already there
			return 0; 
		case START: // Start state cost is already computed
			return startDist_;
		case INTERIOR: // Retrieve the cached value
			return computeMinDistToTerminal(newState);
		default:
			throw new RuntimeException("IMPOSSIBLE");
		}
	}
	
	// This is the value of the cost function c(s,s')
	public int getCost(FSMState oldState, FSMState newState) {
		return 1; // All transitions in the FSM have an equal cost
	}
	
	@Override
	public VerbFSM remap(Map<String, String> nameMap) {
		DirectedGraph<FSMNode, FSMTransition> newDFA = new DirectedSparseGraph<FSMNode, FSMTransition>();
		
		HashMap<FSMNode, FSMNode> stateMap = Maps.newHashMap();
		for (FSMTransition ot : dfa_.getEdges()) {
			FSMNode os = dfa_.getSource(ot);
			if (!stateMap.containsKey(os)) {
				stateMap.put(os, new FSMNode(os.getType()));
			}
			
			FSMNode od = dfa_.getDest(ot);
			if (!stateMap.containsKey(od)) {
				stateMap.put(od, new FSMNode(od.getType()));
			}
			
			newDFA.addEdge(ot.remap(nameMap), stateMap.get(os), stateMap.get(od));
		}
		
		return new VerbFSM(newDFA);
	}

	public void toDot(String file, boolean edgeProb) {
		try {
			BufferedWriter out = new BufferedWriter(new FileWriter(file));
			out.write("digraph G { \n");
			out.write("\tgraph [ rankdir=LR ]; \n");

			for (FSMNode vertex : dfa_.getVertices()) {
				out.write(vertex.toDot());

				double total = 0.0D;
				for (FSMTransition e : dfa_.getOutEdges(vertex)) {
					total += e.count();
				}

				for (FSMTransition e : dfa_.getOutEdges(vertex)) {
					FSMNode end = (FSMNode) dfa_.getDest(e);
					double prob = e.count() / total;

					out.write("\t\"" + vertex.getID() + "\" -> \"" + end.getID()
							+ "\"" + e.toDot(edgeProb, prob) + ";\n");
				}

			}

			out.write("}\n");
			out.close();
		} catch (Exception e) {
			e.printStackTrace();
		}
	}
	
	public int computeMinDistToTerminal(FSMState state) {
		int minDist = Integer.MAX_VALUE;

		if (terminalNodes_.isEmpty()) {
//			throw new RuntimeException("GTFO THERE ARE NO TERMINALS!");
//			logger.info("No terminal nodes found, better be a negative FSM.");
			return Integer.MAX_VALUE;
		}
		
		for (FSMNode s : state.getStates()) {
			int sMinDist = computeMinDistToTerminal(s);
			minDist = Math.min(minDist, sMinDist);
		}
		
		return minDist;
	}
	
	public int computeMinDistToTerminal(FSMNode s) {
		if (terminalNodes_.isEmpty()) {
			throw new RuntimeException("GTFO THERE ARE NO TERMINALS!");
		}
		
		if (!s.hasMinDist()) { // If the minDist for s has not yet been computed, compute it
			UnweightedShortestPath<FSMNode, FSMTransition> pathFinder = new UnweightedShortestPath<FSMNode, FSMTransition>(dfa_);
			int minDist = Integer.MAX_VALUE;
			
			for (FSMNode gt : terminalNodes_) {
				Map<FSMNode, Number> distanceMap = pathFinder.getDistanceMap(s); 
				if (distanceMap.containsKey(gt)) {
					int distance = (Integer) distanceMap.get(gt);
					minDist = Math.min(distance, minDist);
				}
			}
			
			if (minDist == Integer.MAX_VALUE) {
				s.setMinDist(startDist_ + 1); // Transition back to the start state
			} else {
				s.setMinDist(minDist);
			}
		}
		
		return s.getMinDist();
	}
	
	public void reset() {
		activeState_ = startState_;
	}
	
	public FSMState getActiveState() {
		return activeState_;
	}

	private void populateStartState() {
		HashSet<FSMNode> startNodes = new HashSet<FSMNode>();
		for (FSMNode state : dfa_.getVertices()) {
			getMinDistToTerminal(state); // So that the values are pre-cached

			if (state.getType().equals(StateType.START)) {
				startNodes.add(state);
			}
		}
		startState_ = new FSMState(startNodes);
		
		if (dfa_.getVertexCount() == 1) {
			startDist_ = 0;
		} else {
			startDist_ = Integer.MAX_VALUE;
			for (FSMNode node : startNodes) {
				int minDistToTerminal = computeMinDistToTerminal(node);
				startDist_ = Math.min(startDist_, minDistToTerminal);
			}
		}
		
//		startDist_ = getMinDistToTerminal(startState_);
//		startDist_ = startState_.
	}
	
	private void populateTerminals() {
		terminalNodes_ = new HashSet<FSMNode>();
		for (FSMNode state : dfa_.getVertices()) {
			if (state.getType().equals(StateType.TERMINAL)) {
				terminalNodes_.add(state);
			}
		}
	}
	
	public TransitionResult simulateDfaTransition(FSMState state, Set<String> activeProps) {
		FSMNode singleState = null;
		if (state.getStates().size() != 1) {
			throw new IllegalArgumentException("DFA must be in a single state");
		} else {
			singleState = state.getStates().first();
		}
		
		TransitionResult result = new TransitionResult();
		
		FSMNode newNode = null;
		
		if (singleState.getType().equals(StateType.TERMINAL)) {
			newNode = singleState; // TERMINAL cannot be escaped
			throw new RuntimeException("YOU ARE ALREADY AT A TERMINAL, THERE IS NOWHERE TO GO!");
		} else {
			// One edge's active set could be a subset of another's.
			// That would essentially create nondeterminism whenever the superset is present. 
			// To ensure determinism, we can take the most specific transition that matches.
			// That is, simply, the one with the most propositions.
			List<FSMTransition> possibleTransitions = new Vector<FSMTransition>();
			for (FSMTransition e : dfa_.getOutEdges(singleState)) {
				if (e.accept(activeProps)) {
					possibleTransitions.add(e); 
				}
			}
			
			switch (possibleTransitions.size()) {
			case 0:
				// No forward transitions, but...
				// We might be able to loop on the current state, so we need to check the incoming edges
				for (FSMTransition e : dfa_.getInEdges(singleState)) {
					if (e.accept(activeProps)) {
						newNode = singleState;
					}
				}
				// If not, we have gone off the graph, go back to start
				break;
				
			case 1:
				// Simple transition
				newNode = dfa_.getDest(possibleTransitions.get(0));
				break;
				
			default:
				// Descending order by size since we want the most specific first
				Collections.sort(possibleTransitions, new Comparator<FSMTransition>() {
					@Override
					public int compare(FSMTransition arg0, FSMTransition arg1) {
						return arg1.getSymbol().size() - arg0.getSymbol().size(); 
					}
				});
				newNode = dfa_.getDest(possibleTransitions.get(0));
			}
		}
		
		if (newNode == null) {
			result.newState = startState_;
		} else {
			result.newState = new FSMState(newNode);
		}
		result.cost = getCost(state, result.newState);
		result.heuristic = computeMinDistToTerminal(result.newState);
		
		return result;
	}
	
	public TransitionResult simulateNfaTransition(FSMState state, Set<String> activeProps) {
		TransitionResult result = new TransitionResult();
		
		if (state.containsTerminal()) {
			throw new RuntimeException("Planner is broken: simulating leaving a goal state");
		} else {
			Set<FSMNode> newState = new HashSet<FSMNode>();
			List<FSMTransition> transitions = new Vector<FSMTransition>();
			for (FSMNode s : state.getStates()) {
				for (FSMTransition e : dfa_.getOutEdges(s)) {
					if (e.accept(activeProps)) {
						transitions.add(e); 
					}
				}
				for (FSMTransition e : dfa_.getInEdges(s)) {
					if (e.accept(activeProps)) {
						transitions.add(e);
					}
				}
			}
			for (FSMTransition t : transitions) {
				newState.add(dfa_.getDest(t));
			}

			// The start state is "free"
			newState.addAll(startState_.getStates());
			
//			if (!newState.isEmpty()) {
				// We have gone off the graph, go back to start
//			} else {
				result.newState = new FSMState(newState);
//			}
		}
		
		result.cost = getCost(state, result.newState);
		result.heuristic = computeMinDistToTerminal(result.newState);
		
		return result;
	}
	
	public Set<FSMNode> getTerminalStates() {
		return new HashSet<FSMNode>(Collections2.filter(dfa_.getVertices(), FSMNode.isTerminal));
	}
	
	// Prevents adding duplicate edges, since we are in a DFA!
	public void safeAddTransition(FSMNode source, Set<String> symbol, FSMNode dest) {
		if (!StateMachines.hasTransition(dfa_, source, symbol, dest)) {
			dfa_.addEdge(new FSMTransition(symbol), source, dest);
		} 
	}
	
//	private void minimize() {
//		StateMachines.minimizeSlow(dfa_);
//	}
	
	private void printStateValues() {
		for (FSMNode s : dfa_.getVertices()) {
			System.out.println(s + ": " + computeMinDistToTerminal(new FSMState(s)));
		}
	}
	
	/* Verb Composition */ 
	
	// Returns a new FSM equal to this
	public VerbFSM duplicate() {
		return new VerbFSM(StateMachines.duplicate(dfa_));
	}

	// Alternation: Returns a new FSM equal to (this)|(other)
	public VerbFSM alternate(VerbFSM other) {
		DirectedGraph<FSMNode,FSMTransition> first = StateMachines.duplicate(dfa_);
		final DirectedGraph<FSMNode,FSMTransition> second = other.dfa_; // Do not destroy!
		
		// Add all the edges from the second FSM to the first
		HashMap<FSMNode, FSMNode> stateMap = Maps.newHashMap();
		for (FSMTransition t : second.getEdges()) {
			FSMNode source = second.getSource(t);
			if (!stateMap.containsKey(source)) {
				if (source.getType().equals(StateType.START)) {
					stateMap.put(source, Iterables.getOnlyElement(StateMachines.getStartState(first)));
				} else {
					stateMap.put(source, new FSMNode(source.getType()));
				}
			}
			
			FSMNode dest = second.getDest(t);
			if (!stateMap.containsKey(dest)) {
				stateMap.put(dest, new FSMNode(dest.getType()));
			}
			
			first.addEdge(new FSMTransition(t.getActiveRelations()), stateMap.get(source), stateMap.get(dest));
		}
		
		// Now, it may be an NFA, so we need to do subset construction and re-minimize
		DirectedGraph<FSMNode, FSMTransition> dfa = StateMachines.subsetConstruction(first);
		StateMachines.minimizeSlow(dfa);
		return new VerbFSM(dfa);
	}
	
	// Concatenation: Returns a new FSM equal to (this)(other)
	public VerbFSM concatenate(VerbFSM other) {
		if (terminalNodes_.size() > 1) {
			throw new RuntimeException("MINIMIZATION FAILED.");
		}
		
		DirectedGraph<FSMNode, FSMTransition> combined = new DirectedSparseGraph<FSMNode, FSMTransition>();
		
		// Let's try this out
		FSMNode bridge = Iterables.getOnlyElement(terminalNodes_);
		
		// Add all edges from this FSM
		HashMap<FSMNode, FSMNode> stateMap1 = Maps.newHashMap();
		for (FSMTransition ot : dfa_.getEdges()) {
			FSMNode os = dfa_.getSource(ot);
			if (!stateMap1.containsKey(os)) {
				stateMap1.put(os, new FSMNode(os.getType()));
			}
			
			FSMNode od = dfa_.getDest(ot);
			if (!stateMap1.containsKey(od)) {
				stateMap1.put(od, new FSMNode(od.getType()));
			}
			
			combined.addEdge(new FSMTransition(ot.getActiveRelations()), stateMap1.get(os), stateMap1.get(od));
		}
		
		// Add all edges from the next FSM, equating the start state to the old good terminal
		HashMap<FSMNode, FSMNode> stateMap2 = Maps.newHashMap();
		for (FSMTransition ot : other.dfa_.getEdges()) {
			FSMNode os = other.dfa_.getSource(ot);
			
			if (!stateMap2.containsKey(os)) {
				if (os.getType().equals(StateType.START)) {
					stateMap2.put(os, stateMap1.get(bridge));
				} else {
					stateMap2.put(os, new FSMNode(os.getType()));
				}
			}
			
			FSMNode od = other.dfa_.getDest(ot);
			if (!stateMap2.containsKey(od)) {
				stateMap2.put(od, new FSMNode(od.getType()));
			}
			
			combined.addEdge(new FSMTransition(ot.getActiveRelations()), stateMap2.get(os), stateMap2.get(od));
		}
		
		stateMap1.get(bridge).setType(StateType.INTERIOR);
		
		// Add self-loop to bridge
		Vector<FSMTransition> inEdges = new Vector<FSMTransition>(combined.getInEdges(stateMap1.get(bridge)));
		for (FSMTransition inEdge : inEdges) {
			combined.addEdge(new FSMTransition(inEdge.getActiveRelations()), stateMap1.get(bridge), stateMap1.get(bridge));
		}
		
		VerbFSM fusion = new VerbFSM(combined);
		fusion.populateTerminals();
		fusion.toDot("fusion.dot", false);
		
		return fusion; 
	}
	
}
