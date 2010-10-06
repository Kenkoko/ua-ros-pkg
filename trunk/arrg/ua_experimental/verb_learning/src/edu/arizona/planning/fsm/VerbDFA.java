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

import edu.arizona.cs.learn.algorithm.markov.BPPNode;
import edu.arizona.cs.learn.util.graph.Edge;
import edu.arizona.planning.fsm.FSMState.StateType;
import edu.arizona.util.GraphUtils;
import edu.uci.ics.jung.algorithms.shortestpath.UnweightedShortestPath;
import edu.uci.ics.jung.graph.DirectedGraph;

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
		dfa_ = GraphUtils.convertToDFA(graph);
		addSelfLoops();
		populateGoodTerminals();
		
		for (FSMState state : dfa_.getVertices()) {
			if (state.getType().equals(StateType.START)) {
				startState_ = state;
			}
		}
		startDist_ = getMinDistToGoodTerminal(startState_);
		// TODO: Sanity check this
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
			if (e.satisfied(activeProps)) {
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
					return arg1.props().size() - arg0.props().size(); 
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
	
	private void addSelfLoops() {
		for (FSMState state : dfa_.getVertices()) {
			if (state.getType().equals(StateType.GOOD)) {
				for (FSMTransition inEdge : dfa_.getInEdges(state)) {
					dfa_.addEdge(new FSMTransition(inEdge.props()), state, state);
				}
			}
		}
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
}