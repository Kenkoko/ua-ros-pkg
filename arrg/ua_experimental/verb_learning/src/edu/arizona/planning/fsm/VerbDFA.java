package edu.arizona.planning.fsm;

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.util.Collection;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.Vector;

import com.google.common.base.Function;
import com.google.common.collect.Collections2;
import com.google.common.collect.Sets;

import edu.arizona.cs.learn.algorithm.markov.BPPNode;
import edu.arizona.cs.learn.util.graph.Edge;
import edu.arizona.planning.fsm.FSMState.StateType;
import edu.arizona.util.Predicate;
import edu.arizona.util.StateMachines;
import edu.uci.ics.jung.algorithms.shortestpath.UnweightedShortestPath;
import edu.uci.ics.jung.graph.DirectedGraph;
import edu.uci.ics.jung.graph.util.Pair;

public class VerbDFA {
	public class SimulationResult {
		public FSMState newState = null;
		public double cost = 0.0;
	}
	
	private FSMState activeState_;
	private DirectedGraph<FSMState, FSMTransition> dfa_;
	private FSMState startState_;
	
	private Set<FSMState> goodTerminals_;
	private int startDist_;
	
	public VerbDFA(DirectedGraph<BPPNode, Edge> graph) {
		// Preparing the DFA
		dfa_ = StateMachines.convertToDFA(graph);
		minimizeSlow();
		addSelfLoops();
		populateGoodTerminals();
		
		for (FSMState state : dfa_.getVertices()) {
			if (state.getType().equals(StateType.START)) {
				startState_ = state;
			}
		}
		startDist_ = getMinDistToGoodTerminal(startState_);

		activeState_ = startState_;
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
	// REWARD IS BEING USED AS COST FOR NOW IT'S ALL FUCKED UP
	public SimulationResult simulate(FSMState state, Set<String> activeProps) {
		SimulationResult result = new SimulationResult();
		
		// One edge's active set could be a subset of another's.
		// That would essentially create nondeterminism whenever the superset is present. 
		// To ensure determinism, we can take the most specific transition that matches.
		// That is, simply, the one with the most propositions.
		List<FSMTransition> possibleTransitions = new Vector<FSMTransition>();
		for (FSMTransition e : dfa_.getOutEdges(state)) {
//			System.out.println("PART 1: " + activeProps);
//			System.out.println("PART 2: " + e.props());
//			System.out.println("PART 3: " + e.satisfied(activeProps));
			if (e.accept(activeProps)) {
				possibleTransitions.add(e); 
			}
		}
		
		switch (possibleTransitions.size()) {
		case 0:
			// We have gone off the graph, go back to start
			result.newState = startState_;
			break;
		case 1:
			// Simple transition
			result.newState = dfa_.getDest(possibleTransitions.get(0));
			break;
		default:
			// Descending order by size since we want the most specific first
			Collections.sort(possibleTransitions, new Comparator<FSMTransition>() {
				@Override
				public int compare(FSMTransition arg0, FSMTransition arg1) {
					return arg1.getSymbol().size() - arg0.getSymbol().size(); 
				}
			});
			result.newState = dfa_.getDest(possibleTransitions.get(0));
		}

		result.cost = getCost(state, result.newState);
		
		return result;
	}
	
	// This returns the reward now
	public double update(Set<String> activeProps) {
		SimulationResult result = simulate(activeState_, activeProps);
		activeState_ = result.newState;
		return result.cost;
	}
	
	public double getCost(FSMState oldState, FSMState newState) {
		switch (newState.getType()) {
		case GOOD_TERMINAL: // No cost
			return 0.0; 
		case START: // Start state cost is already computed
			return startDist_;
		case BAD_TERMINAL: // YOU BE DEAD!
			return Double.POSITIVE_INFINITY; // startDist_ * 10;
		case GOOD: // For all other states 
			return getMinDistToGoodTerminal(newState);
		default:
			throw new RuntimeException("IMPOSSIBLE");
		}
	}
	
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
	
	public FSMState getActiveState() {
		return activeState_;
	}
	
	public FSMState getStartState() {
		return startState_;
	}
	
	
	
	public void reset() {
		activeState_ = startState_;
	}
	
	public int getMinDistToGoodTerminal(FSMState state) {
		UnweightedShortestPath<FSMState, FSMTransition> pathFinder = new UnweightedShortestPath<FSMState, FSMTransition>(dfa_);
		// TODO: Probably should cache these in the states
		Map<FSMState, Number> distanceMap = pathFinder.getDistanceMap(state); 
		
		int minDist = Integer.MAX_VALUE;
		
		for (FSMState gt : goodTerminals_) {
			if (distanceMap.containsKey(gt)) {
				int distance = (Integer) distanceMap.get(gt);
				minDist = Math.min(distance, minDist);
			}
		}
		
		if (minDist == Integer.MAX_VALUE) {
			minDist = startDist_ + 1; // 1 for going back to the start state, then the start state's cost
		}
		
		return minDist;
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
	
	// General state machine stuff
	
	private void addSelfLoops() {
		for (FSMState state : dfa_.getVertices()) {
			if (state.getType().equals(StateType.GOOD)) {
				for (FSMTransition inEdge : dfa_.getInEdges(state)) {
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
		if (!hasTransition(dfa_, source, symbol, dest)) {
			dfa_.addEdge(new FSMTransition(symbol), source, dest);
		} 
	}
	
	/***
	 * Minimize DFA using simpler O(n^2) algorithm. 
	 */
	private void minimizeSlow() {
		Vector<FSMState> states = new Vector<FSMState>(dfa_.getVertices());
		
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
					
					Set<FSMTransition> pEdges = Sets.newHashSet(dfa_.getOutEdges(p));
					Set<FSMTransition> qEdges = Sets.newHashSet(dfa_.getOutEdges(q));
					Set<Set<String>> pSymbols = Sets.newHashSet(Collections2.transform(pEdges, getSymbol));
					Set<Set<String>> qSymbols = Sets.newHashSet(Collections2.transform(qEdges, getSymbol));
					if (pSymbols.equals(qSymbols)) {
						for (FSMTransition t : pEdges) {
							FSMState pPrime = dfa_.getOpposite(p, t);
							FSMState qPrime = dfa_.getOpposite(q, matchTransition(dfa_, q, t.getSymbol()));
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
		
		System.out.println(distinct);
		
		// Now that we know which are distinguishable, we need to delete the unnecessary states
		for (Pair<FSMState> same : Sets.difference(allPairs, distinct)) {
			FSMState saved = same.getFirst();
			FSMState dead = same.getSecond();
			
			for (FSMTransition t : dfa_.getInEdges(dead)) {
				FSMState p = dfa_.getOpposite(dead, t);
				safeAddTransition(p, t.getSymbol(), saved);
			}
			for (FSMTransition t : dfa_.getOutEdges(dead)) {
				FSMState p = dfa_.getOpposite(dead, t);
				safeAddTransition(saved, t.getSymbol(), p);
			}
			
			// It's all over for him
			for (FSMTransition t : dfa_.getIncidentEdges(dead)) {
				dfa_.removeEdge(t);
			}
			dfa_.removeVertex(dead);
		}
	}
	
	private boolean hasTransition(DirectedGraph<FSMState, FSMTransition> dfa, FSMState source, Set<String> symbol, FSMState dest) {
		FSMTransition t = matchTransition(dfa, source, symbol);
		if (t == null) {
			return false;
		} else {
			return dfa.getOpposite(source, t).equals(dest);
		}
	}
	
	private FSMTransition matchTransition(DirectedGraph<FSMState, FSMTransition> dfa, FSMState state, final Set<String> symbol) {
		Collection<FSMTransition> matchingTransitions = Collections2.filter(dfa.getOutEdges(state), new Predicate<FSMTransition>() {
			public boolean value(FSMTransition arg) { return arg.getSymbol().equals(symbol); }
		});
		if (matchingTransitions.isEmpty()) {
			return null;
		} else {
			return matchingTransitions.iterator().next();
		}
	}
}