package edu.arizona.planning.fsm;

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;
import java.util.Set;
import java.util.Vector;

import edu.arizona.cs.learn.algorithm.markov.BPPNode;
import edu.arizona.cs.learn.util.graph.Edge;
import edu.arizona.planning.fsm.FSMState.StateType;
import edu.arizona.util.GraphUtils;
import edu.uci.ics.jung.graph.DirectedGraph;

public class VerbDFA {
	public class SimulationResult {
		public FSMState newState = null;
		public double cost = 0.0;
	}
	
	private FSMState activeState_;
	private DirectedGraph<FSMState, Edge> dfa_;
	private FSMState startState_;
	
	public VerbDFA(DirectedGraph<BPPNode, Edge> graph) {
		if (!GraphUtils.isDFA(graph)) {
			throw new RuntimeException("GRAPH IS NOT A DFA");
			// TODO: In future, obviously this should just convert it to a DFA
		}
		dfa_ = GraphUtils.convertToFSM(graph);
		
		for (FSMState state : dfa_.getVertices()) {
			if (state.getType().equals(StateType.START)) {
				startState_ = state;
			}
		}
		activeState_ = startState_;
	}

	// The cycles that are implicit should have been made explicit, so this will all be very clean
	// REWARD IS BEING USED AS COST FOR NOW IT'S ALL FUCKED UP
	public SimulationResult simulate(FSMState state, Set<String> activeProps) {
//		FSMState oldState = state;
		SimulationResult result = new SimulationResult();
		
		// One edge's active set could be a subset of another's.
		// That would essentially create nondeterminism whenever the superset is present. 
		// To ensure determinism, we can take the most specific transition that matches.
		// That is, simply, the one with the most propositions.
		List<Edge> possibleTransitions = new Vector<Edge>();
		for (Edge e : dfa_.getOutEdges(state)) {
//			System.out.println("PART 1: " + activeProps);
//			System.out.println("PART 2: " + e.props());
//			System.out.println("PART 3: " + e.satisfied(activeProps));
			if (e.satisfied(activeProps)) {
				possibleTransitions.add(e); 
			}
		}
		
//		System.out.println(possibleTransitions);
		
		switch (possibleTransitions.size()) {
		case 0:
			// We have gone off the graph, go back to start
			result.newState = startState_;
			result.cost = 2.0;
			return result;
		case 1:
			// Simple transition
			result.newState = dfa_.getDest(possibleTransitions.get(0));
			break;
		default:
			Collections.sort(possibleTransitions, new Comparator<Edge>() {
				@Override
				public int compare(Edge arg0, Edge arg1) {
					return arg1.props().size() - arg0.props().size(); // Descending order by size since we want the most specific first
				}
			});
			result.newState = dfa_.getDest(possibleTransitions.get(0));
		}

		switch (result.newState.getType()) {
		case GOAL:
			result.cost = 0.0; //1.0;
			break;
		case GOOD:
			if (result.newState.equals(state)) { // We stayed in the same state
				result.cost = 0.3; // -0.1;
			} else if (dfa_.isSuccessor(state, result.newState)) { // We moved forward
				result.cost = 0.01;
			} else {
				throw new RuntimeException("THERE IS NO WAY BACK");
			}
			break;
		case BAD:
			result.cost = 4; // -1.0;
			break;
		case BAD_TERMINAL:
			result.cost = 10; //-2.0;
			break;
		case START:
		default:
			throw new RuntimeException("IMPOSSIBLE");
		}
		
		return result;
	}
	
	// This returns the reward now
	public double update(Set<String> activeProps) {
		SimulationResult result = simulate(activeState_, activeProps);
		activeState_ = result.newState;
		return result.cost;
	}
	
	public void addConstraintState(Set<String> bannedProps) {
		FSMState deadState = new FSMState(StateType.BAD_TERMINAL);
		
		dfa_.addVertex(deadState);
		for (FSMState state : dfa_.getVertices()) {
			if (state.getType().equals(StateType.GOOD) || state.getType().equals(StateType.START)) {
				Edge e = new Edge(bannedProps);
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
	
	public void toDot(String file, boolean edgeProb) {
		try {
			BufferedWriter out = new BufferedWriter(new FileWriter(file));
			out.write("digraph G { \n");
			out.write("\tgraph [ rankdir=LR ]; \n");

			for (FSMState vertex : dfa_.getVertices()) {
				out.write(vertex.toDot());

				double total = 0.0D;
				for (Edge e : dfa_.getOutEdges(vertex)) {
					total += e.count();
				}

				for (Edge e : dfa_.getOutEdges(vertex)) {
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