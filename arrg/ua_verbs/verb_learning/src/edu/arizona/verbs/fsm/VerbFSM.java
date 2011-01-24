package edu.arizona.verbs.fsm;

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.util.ArrayList;
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
import com.google.common.collect.Sets;

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
	private FSMState activeState_ = new FSMState();
	private Set<FSMNode> terminalNodes_;
	private int startDist_;
	
	private ArrayList<FSMState> startStates_ = new ArrayList<FSMState>();
	
	private VerbFSM(DirectedGraph<FSMNode, FSMTransition> dfa) {
		dfa_ = dfa;
		init();
	}
	
	public VerbFSM(Set<CorePath> corePaths) {
		dfa_ = StateMachines.createFSM(corePaths);
		// This distances are potentially invalid because we don't know how the FSM was made
		for (FSMNode n : dfa_.getVertices()) { 
			n.clearMinDist();
		}
		init();
	}

	private void init() {
		populateTerminals(); 	
		populateStartState();
		
		activeState_ = startStates_.get(0);
		
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
	
	public FSMState getGlobalStartState() {
		return startStates_.get(0);
	}
	
	public FSMState getStartState(int subverbIndex) {
		return startStates_.get(subverbIndex);
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
			return minDist; // Return the max value
		}
		
		for (FSMNode s : state.getStates()) {
			minDist = Math.min(minDist, computeMinDistToTerminal(s));
		}
		
		return minDist;
	}
	
	public int computeMinDistToTerminal(FSMNode s) {
		if (terminalNodes_.isEmpty()) {
			throw new RuntimeException("GTFO THERE ARE NO TERMINALS!");
		}
		
//		if (!s.hasMinDist()) { // If the minDist for s has not yet been computed, compute it
//			UnweightedShortestPath<FSMNode, FSMTransition> pathFinder = new UnweightedShortestPath<FSMNode, FSMTransition>(dfa_);
//			int minDist = Integer.MAX_VALUE;
//			
//			for (FSMNode gt : terminalNodes_) {
//				Map<FSMNode, Number> distanceMap = pathFinder.getDistanceMap(s); 
//				if (distanceMap.containsKey(gt)) {
//					int distance = (Integer) distanceMap.get(gt);
//					minDist = Math.min(distance, minDist);
//				}
//			}
//			
//			if (minDist == Integer.MAX_VALUE) {
//				s.setMinDist(startDist_ + 1); // Transition back to the start state
//			} else {
//				s.setMinDist(minDist);
//			}
//		}
		
		int minDist = StateMachines.findMinDistToTerminal(dfa_, s);
		s.setMinDist(minDist);
		
		if (s.getType().equals(StateType.INTERIOR) && s.getMinDist() == 0) {
			throw new RuntimeException("IMPOSSIBLE: Interior node has 0 distance to terminal");
		}
		
//		return s.getMinDist();
		return minDist;
	}
	
	public void reset() {
		activeState_ = startStates_.get(0);
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
		if (startStates_.isEmpty()) {
			startStates_.add(new FSMState(startNodes)); // The first one is always the global start
		}
		
		if (dfa_.getVertexCount() == 1) {
			startDist_ = 0;
		} else {
			startDist_ = Integer.MAX_VALUE;
			for (FSMNode node : startNodes) {
				int minDistToTerminal = computeMinDistToTerminal(node);
				startDist_ = Math.min(startDist_, minDistToTerminal);
			}
		}
	}
	
	private void populateTerminals() {
		terminalNodes_ = new HashSet<FSMNode>();
		for (FSMNode state : dfa_.getVertices()) {
			if (state.getType().equals(StateType.TERMINAL)) {
				terminalNodes_.add(state);
			}
		}
	}
	
	public TransitionResult simulateDfaHelper(FSMState state, Set<String> activeProps) {
		FSMNode node = null;
		if (state.getStates().size() != 1) {
			throw new IllegalArgumentException("DFA must be in a single state");
		} else {
			node = state.getStates().first();
		}
		
		TransitionResult result = new TransitionResult();
		
		FSMNode newNode = null;
		
		if (node.getType().equals(StateType.TERMINAL)) {
			newNode = node; // TERMINAL cannot be escaped
			throw new RuntimeException("YOU ARE ALREADY AT A TERMINAL, THERE IS NOWHERE TO GO!");
			
		} else {
			// One edge's active set could be a subset of another's.
			// That would essentially create nondeterminism whenever the superset is present. 
			// To ensure determinism, we can take the most specific transition that matches.
			// That is, simply, the one with the most propositions.
			List<FSMTransition> possibleTransitions = new Vector<FSMTransition>();
			for (FSMTransition e : dfa_.getOutEdges(node)) {
				if (e.accept(activeProps)) {
					possibleTransitions.add(e); 
				}
			}
			
			switch (possibleTransitions.size()) {
			case 0:
				// No forward transitions, but...
				// We might be able to loop on the current state, so we need to check the incoming edges
				for (FSMTransition e : dfa_.getInEdges(node)) {
					if (e.accept(activeProps)) {
						newNode = node;
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
			result.newState = startStates_.get(node.getVerbSequenceIndex()); // Go back to the right start state
		} else {
			result.newState = new FSMState(newNode);
		}
		result.cost = getCost(state, result.newState);
		result.heuristic = computeMinDistToTerminal(result.newState);
		
		return result;
	}
	
	public boolean isaStart(FSMState state) {
		return startStates_.contains(state);
	}
	
	public TransitionResult simulateDfaTransition(FSMState state, Set<String> activeProps) {
		TransitionResult firstResult = simulateDfaHelper(state, activeProps);
		
//		return firstResult;
		if (isaStart(firstResult.newState)) { // "push through" start states
			TransitionResult secondResult = simulateDfaHelper(firstResult.newState, activeProps);
			secondResult.cost = getCost(state, secondResult.newState); // Tricky
			return secondResult;
		} else {
			return firstResult;
		}
	}
	
	
	
	
	
	// TODO this doesn't support multiple start states, which is OK for now
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
			newState.addAll(startStates_.get(0).getStates());
			
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
		System.out.println("STATE VALUES: ");
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
	
	public int getTerminalIndex() {
		if (terminalNodes_.isEmpty()) {
			return 0;
		} else {
			return terminalNodes_.iterator().next().getVerbSequenceIndex();
		}
	}
	
	// Concatenation: Returns a new FSM equal to (this)(other)
	// TODO nested arbitrary concatenation is not supported yet
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
				stateMap1.put(os, new FSMNode(os.getType(), this.getTerminalIndex()));
			}
			
			FSMNode od = dfa_.getDest(ot);
			if (!stateMap1.containsKey(od)) {
				stateMap1.put(od, new FSMNode(od.getType(), this.getTerminalIndex()));
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
					stateMap2.put(os, new FSMNode(os.getType(), this.getTerminalIndex() + 1));
				}
			}
			
			FSMNode od = other.dfa_.getDest(ot);
			if (!stateMap2.containsKey(od)) {
				stateMap2.put(od, new FSMNode(od.getType(), this.getTerminalIndex() + 1));
			}
			
			combined.addEdge(new FSMTransition(ot.getActiveRelations()), stateMap2.get(os), stateMap2.get(od));
		}
		
		stateMap1.get(bridge).setType(StateType.INTERIOR);
		stateMap1.get(bridge).setVerbSequenceIndex(this.getTerminalIndex() + 1);
		
		VerbFSM fusion = new VerbFSM(combined);
		fusion.populateTerminals();
		
		fusion.startStates_.clear();
		// Add all of my start states
		for (FSMState start : startStates_) {
			fusion.startStates_.add(new FSMState(stateMap1.get(Iterables.getOnlyElement(start.getStates()))));
		}
		// Then add the new start state
		fusion.startStates_.add(new FSMState(stateMap1.get(bridge)));
		
		System.out.println(fusion.startStates_);
		
		fusion.toDot("fusion.dot", false);
		
		return fusion; 
	}
	
}
