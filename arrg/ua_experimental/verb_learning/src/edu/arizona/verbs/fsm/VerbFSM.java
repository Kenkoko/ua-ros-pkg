package edu.arizona.verbs.fsm;

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.util.Collection;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.TreeSet;
import java.util.Vector;

import com.google.common.base.Function;
import com.google.common.collect.Collections2;
import com.google.common.collect.Iterables;
import com.google.common.collect.Maps;
import com.google.common.collect.Sets;
import com.google.common.collect.Sets.SetView;

import edu.arizona.cs.learn.algorithm.markov.BPPNode;
import edu.arizona.cs.learn.util.graph.Edge;
import edu.arizona.verbs.fsm.FSMState.StateType;
import edu.arizona.verbs.shared.Remappable;
import edu.arizona.verbs.util.Graphs;
import edu.uci.ics.jung.algorithms.shortestpath.UnweightedShortestPath;
import edu.uci.ics.jung.graph.DirectedGraph;
import edu.uci.ics.jung.graph.DirectedSparseGraph;
import edu.uci.ics.jung.graph.util.Pair;

public class VerbFSM implements Remappable<VerbFSM> {
	
	public class TransitionResult {
		public TreeSet<FSMState> newState = new TreeSet<FSMState>();
		public double cost = 0.0;
		public double heuristic = 0.0;
	}
	
	DirectedGraph<FSMState, FSMTransition> dfa_;
	TreeSet<FSMState> startState_ = new TreeSet<FSMState>();
	TreeSet<FSMState> activeState_ = new TreeSet<FSMState>();
	Set<FSMState> goodTerminals_;
	int startDist_;
	
	public VerbFSM(DirectedGraph<BPPNode, Edge> positiveChains, DirectedGraph<BPPNode, Edge> negativeChains) {
		// Convert NFA to DFA
		DirectedGraph<FSMState, FSMTransition> positiveDFA = StateMachines.convertToDFA(positiveChains, true);
		DirectedGraph<FSMState, FSMTransition> negativeDFA = StateMachines.convertToDFA(negativeChains, false);
		dfa_ = StateMachines.combineDFAs(positiveDFA, negativeDFA);
		
		if (!Graphs.isDFA(dfa_)) { throw new RuntimeException("SUBSET CONSTRUCTION FAILED"); }
		toDot("1.dot", false);
		
		// Add self loops
		addSelfLoops();
		if (!Graphs.isDFA(dfa_)) { throw new RuntimeException("SELF LOOPS FAILED"); }
		toDot("3.dot", false);
		
		// Minimize the DFA - let's see what happens now
		minimize();
		if (!Graphs.isDFA(dfa_)) { throw new RuntimeException("MINIMIZATION FAILED"); }
		toDot("2.dot", false);
		
		populateGoodTerminals(); // This doesn't change the graph structure
		
		for (FSMState state : dfa_.getVertices()) {
			getMinDistToGoodTerminal(state); // So that the values are pre-cached

			if (state.getType().equals(StateType.START)) {
				startState_.add(state);
			}
		}
		
		startDist_ = getMinDistToGoodTerminal(Sets.newTreeSet(startState_));
		
		activeState_ = startState_;
		
		printStateValues();
	}

	private VerbFSM(DirectedGraph<FSMState, FSMTransition> dfa) {
		dfa_ = dfa;
		
		// TODO: How much of the stuff below do we really need
		populateGoodTerminals(); // This doesn't change the graph structure
		
		for (FSMState state : dfa_.getVertices()) {
			getMinDistToGoodTerminal(state); // So that the values are pre-cached

			if (state.getType().equals(StateType.START)) {
				startState_.add(state);
			}
		}
		
		startDist_ = getMinDistToGoodTerminal(Sets.newTreeSet(startState_));
		
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
	
	public TreeSet<FSMState> getStartState() {
		return startState_;
	}
	
	// This is the value of the heuristic function h(s)
	public double getHeuristic(TreeSet<FSMState> newState) {
		switch (StateSet.getStateType(newState)) {
		case GOOD_TERMINAL: // No cost
			return 0.0; 
		case START: // Start state cost is already computed
			return startDist_;
		case BAD_TERMINAL: // YOU BE DEAD!
			return Double.POSITIVE_INFINITY; 
		case GOOD: // For all other states // TODO: Cache these
			return getMinDistToGoodTerminal(newState);
		default:
			throw new RuntimeException("IMPOSSIBLE");
		}
	}
	
	// This is the value of the cost function c(s,a,s')
	public double getCost(TreeSet<FSMState> oldState, TreeSet<FSMState> newState) {
		return 1.0;
	}
	
	@Override
	public VerbFSM remap(Map<String, String> nameMap) {
		DirectedGraph<FSMState, FSMTransition> newDFA = new DirectedSparseGraph<FSMState, FSMTransition>();
		
		HashMap<FSMState, FSMState> stateMap = Maps.newHashMap();
		for (FSMTransition ot : dfa_.getEdges()) {
			FSMState os = dfa_.getSource(ot);
			if (!stateMap.containsKey(os)) {
				stateMap.put(os, new FSMState(os.getType()));
			}
			
			FSMState od = dfa_.getDest(ot);
			if (!stateMap.containsKey(od)) {
				stateMap.put(od, new FSMState(od.getType()));
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

			for (FSMState vertex : dfa_.getVertices()) {
				out.write(vertex.toDot());

				double total = 0.0D;
				for (FSMTransition e : dfa_.getOutEdges(vertex)) {
					total += e.count();
				}

				for (FSMTransition e : dfa_.getOutEdges(vertex)) {
					FSMState end = (FSMState) dfa_.getDest(e);
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
	
	public int getMinDistToGoodTerminal(TreeSet<FSMState> state) {
		int minDist = Integer.MAX_VALUE;

		if (goodTerminals_.isEmpty()) {
			throw new RuntimeException("GTFO THERE ARE NO GOOD TERMINALS!");
		}
		
		for (FSMState s : state) {
			int sMinDist = getMinDistToGoodTerminal(s);
			minDist = Math.min(minDist, sMinDist);
		}
		
		return minDist;
	}
	
	public int getMinDistToGoodTerminal(FSMState s) {
		if (goodTerminals_.isEmpty()) {
			throw new RuntimeException("GTFO THERE ARE NO GOOD TERMINALS!");
		}
		
		if (!s.hasMinDist()) { // If the minDist for s has not yet been computed, compute it
			UnweightedShortestPath<FSMState, FSMTransition> pathFinder = new UnweightedShortestPath<FSMState, FSMTransition>(dfa_);
			int minDist = Integer.MAX_VALUE;
			
			for (FSMState gt : goodTerminals_) {
				Map<FSMState, Number> distanceMap = pathFinder.getDistanceMap(s); 
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
	
	public Set<FSMState> getActiveState() {
		return activeState_;
	}

	private void populateGoodTerminals() {
		goodTerminals_ = new HashSet<FSMState>();
		for (FSMState state : dfa_.getVertices()) {
			if (state.getType().equals(StateType.GOOD_TERMINAL)) {
				goodTerminals_.add(state);
			}
		}
	}
	
	// The cycles that are implicit should have been made explicit, so this will all be very clean
	public TransitionResult simulateDfaTransition(TreeSet<FSMState> state, Set<String> activeProps) {
		FSMState singleState = null;
		if (state.size() != 1) {
			throw new IllegalArgumentException("DFA must be in a single state");
		} else {
			singleState = state.iterator().next();
		}
		
		TransitionResult result = new TransitionResult();
		
		FSMState newState = null;
		
		if (singleState.getType().equals(StateType.BAD_TERMINAL)) {
			newState = singleState; // BAD_TERMINAL cannot be escaped
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
				// We have gone off the graph, go back to start
				break;
			case 1:
				// Simple transition
				newState = dfa_.getDest(possibleTransitions.get(0));
				break;
			default:
				// Descending order by size since we want the most specific first
				Collections.sort(possibleTransitions, new Comparator<FSMTransition>() {
					@Override
					public int compare(FSMTransition arg0, FSMTransition arg1) {
						return arg1.getSymbol().size() - arg0.getSymbol().size(); 
					}
				});
				newState = dfa_.getDest(possibleTransitions.get(0));
			}
		}
		
		if (newState == null) {
			result.newState = startState_;
		} else {
			result.newState.add(newState);
		}
		result.cost = getCost(state, result.newState);
		result.heuristic = getHeuristic(result.newState);
		
		return result;
	}
	
	public TransitionResult simulateNfaTransition(TreeSet<FSMState> state, Set<String> activeProps) {
		TransitionResult result = new TransitionResult();
		
		if (StateSet.containsGoodTerminal(state)) {
			throw new RuntimeException("Planner is broken: simulating leaving a goal state");
		} else {
			Set<FSMState> newState = new HashSet<FSMState>();
			List<FSMTransition> transitions = new Vector<FSMTransition>();
			for (FSMState s : state) {
				for (FSMTransition e : dfa_.getOutEdges(s)) {
					if (e.accept(activeProps)) {
						transitions.add(e); 
					}
				}
			}
			for (FSMTransition t : transitions) {
				result.newState.add(dfa_.getDest(t));
			}
			
			if (newState.isEmpty()) {
				// We have gone off the graph, go back to start
				result.newState = Sets.newTreeSet(startState_);
			}
		}
		
		result.cost = getCost(state, result.newState);
		result.heuristic = getHeuristic(result.newState);
		
		return result;
	}
	
	@Deprecated
	public void addConstraintState(Set<String> bannedProps) {
		FSMState deadState = new FSMState(StateType.BAD_TERMINAL);
		
		dfa_.addVertex(deadState);
		for (FSMState state : dfa_.getVertices()) {
			if (state.getType().equals(StateType.GOOD) || state.getType().equals(StateType.START)) {
				FSMTransition e = new FSMTransition(bannedProps);
				dfa_.addEdge(e, state, deadState);
			}
		}
		
		// TODO: Need to ensure that we have not broken DFA property and reconvert if necessary
	}
	
	// General state machine stuff
	
	private void addSelfLoops() {
		for (FSMState state : dfa_.getVertices()) {
			if (state.getType().equals(StateType.GOOD)) {
				// TRICKY: Be careful of concurrent modification when adding edges
				Vector<FSMTransition> inEdges = new Vector<FSMTransition>(dfa_.getInEdges(state));
				for (FSMTransition inEdge : inEdges) {
					dfa_.addEdge(new FSMTransition(inEdge.getSymbol()), state, state);
				}
			}
		}
	}
	
	public Set<FSMState> getTerminalStates() {
		return new HashSet<FSMState>(Collections2.filter(dfa_.getVertices(), FSMState.isTerminal));
	}
	
	// Prevents adding duplicate edges, since we are in a DFA!
	public void safeAddTransition(FSMState source, Set<String> symbol, FSMState dest) {
		if (!StateMachines.hasTransition(dfa_, source, symbol, dest)) {
			dfa_.addEdge(new FSMTransition(symbol), source, dest);
		} 
	}
	
	private void minimize() {
		StateMachines.minimizeSlow(dfa_);
	}
	
	private void printStateValues() {
		for (FSMState s : dfa_.getVertices()) {
			System.out.println(s + ": " + getMinDistToGoodTerminal(Sets.newTreeSet(Sets.newHashSet(s))));
		}
	}
	
	/* Verb Composition */ 
	
	// Returns a new FSM equal to this
	public VerbFSM duplicate() {
		return new VerbFSM(StateMachines.duplicate(dfa_));
	}

	// Alternation: Returns a new FSM equal to (this)|(other)
	public VerbFSM alternate(VerbFSM other) {
		DirectedGraph<FSMState,FSMTransition> first = StateMachines.duplicate(dfa_);
		final DirectedGraph<FSMState,FSMTransition> second = other.dfa_; // Do not destroy!
		
		// Add all the edges from the second FSM to the first
		HashMap<FSMState, FSMState> stateMap = Maps.newHashMap();
		for (FSMTransition t : second.getEdges()) {
			FSMState source = second.getSource(t);
			if (!stateMap.containsKey(source)) {
				if (source.getType().equals(StateType.START)) {
					stateMap.put(source, Iterables.getOnlyElement(StateMachines.getStartState(first)));
				} else {
					stateMap.put(source, new FSMState(source.getType()));
				}
			}
			
			FSMState dest = second.getDest(t);
			if (!stateMap.containsKey(dest)) {
				stateMap.put(dest, new FSMState(dest.getType()));
			}
			
			first.addEdge(new FSMTransition(t.getActiveRelations()), stateMap.get(source), stateMap.get(dest));
		}
		
		// Now, it may be an NFA, so we need to do subset construction and re-minimize
		DirectedGraph<FSMState, FSMTransition> dfa = StateMachines.subsetConstruction(first);
		StateMachines.minimizeSlow(dfa);
		return new VerbFSM(dfa);
	}
	
	// Concatenation: Returns a new FSM equal to (this)(other)
	public VerbFSM concatenate(VerbFSM other) {
		// TODO: Do something smarter in minimize to handle this case properly
		if (goodTerminals_.size() > 1) {
			throw new RuntimeException("MINIMIZATION FAILED.");
		}
		
		DirectedGraph<FSMState, FSMTransition> combined = new DirectedSparseGraph<FSMState, FSMTransition>();
		
		// Let's try this out
		FSMState bridge = Iterables.getOnlyElement(goodTerminals_);
		
		// Add all edges from this FSM
		HashMap<FSMState, FSMState> stateMap1 = Maps.newHashMap();
		for (FSMTransition ot : dfa_.getEdges()) {
			FSMState os = dfa_.getSource(ot);
			if (!stateMap1.containsKey(os)) {
				stateMap1.put(os, new FSMState(os.getType()));
			}
			
			FSMState od = dfa_.getDest(ot);
			if (!stateMap1.containsKey(od)) {
				stateMap1.put(od, new FSMState(od.getType()));
			}
			
			combined.addEdge(new FSMTransition(ot.getActiveRelations()), stateMap1.get(os), stateMap1.get(od));
		}
		
		// Add all edges from the next FSM, equating the start state to the old good terminal
		HashMap<FSMState, FSMState> stateMap2 = Maps.newHashMap();
		for (FSMTransition ot : other.dfa_.getEdges()) {
			FSMState os = other.dfa_.getSource(ot);
			
			if (!stateMap2.containsKey(os)) {
				if (os.getType().equals(StateType.START)) {
					stateMap2.put(os, stateMap1.get(bridge));
				} else {
					stateMap2.put(os, new FSMState(os.getType()));
				}
			}
			
			FSMState od = other.dfa_.getDest(ot);
			if (!stateMap2.containsKey(od)) {
				stateMap2.put(od, new FSMState(od.getType()));
			}
			
			combined.addEdge(new FSMTransition(ot.getActiveRelations()), stateMap2.get(os), stateMap2.get(od));
		}
		
		stateMap1.get(bridge).setType(StateType.GOOD);
		
		// Add self-loop to bridge
		Vector<FSMTransition> inEdges = new Vector<FSMTransition>(combined.getInEdges(stateMap1.get(bridge)));
		for (FSMTransition inEdge : inEdges) {
			combined.addEdge(new FSMTransition(inEdge.getActiveRelations()), stateMap1.get(bridge), stateMap1.get(bridge));
		}
		
		VerbFSM fusion = new VerbFSM(combined);
		fusion.populateGoodTerminals();
		fusion.toDot("fusion.dot", false);
		
		return fusion; 
	}
	
}
