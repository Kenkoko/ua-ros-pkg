package edu.arizona.cs.learn.algorithm.markov;

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Queue;
import java.util.Set;

import org.apache.log4j.Logger;

import edu.arizona.cs.learn.algorithm.bpp.BPPFactory;
import edu.arizona.cs.learn.timeseries.model.Interval;
import edu.arizona.cs.learn.util.graph.Edge;
import edu.uci.ics.jung.graph.DirectedGraph;
import edu.uci.ics.jung.graph.DirectedSparseGraph;

public class FSMFactory {
	private static Logger logger = Logger.getLogger(FSMFactory.class);

	public static DirectedGraph<BPPNode, Edge> makeGraph(List<String> propList, List<List<Interval>> instances, 
			boolean onlyStart) {
		
		DirectedGraph<BPPNode,Edge> graph = new DirectedSparseGraph<BPPNode,Edge>();
		Map<String,BPPNode> vertexes = new HashMap<String,BPPNode>();

		BPPNode startNode = new StringNode("start", null);
		graph.addVertex(startNode);

		for (int k = 0; k < instances.size(); k++) {
			List<Interval> bpp = instances.get(k);

			char[][] timeLine = BPPFactory.timeLine(propList, bpp);
			int time = timeLine[0].length;

			BPPNode lastNode = startNode;
			Set<String> props = new HashSet<String>();
			for (int i = 0; i < time; i++) {
				props = new HashSet<String>();
				StringBuffer state = new StringBuffer();
				for (int j = 0; j < propList.size(); j++) {
					if (onlyStart) {
						if (timeLine[j][i] == '1') {
							char value = '1';
							if ((i > 0) && (timeLine[j][(i - 1)] == '1'))
								value = '0';
							else
								props.add((String) propList.get(j));
							state.append(value);
						} else {
							state.append(timeLine[j][i]);
						}
					} else {
						if (timeLine[j][i] == '1') {
							props.add((String) propList.get(j));
						}
						state.append(timeLine[j][i]);
					}
				}
				
				// Commented out by wkerr on 1/15/2011
				// Turns out that we need these "break" nodes
				// in order to allow the machine to wait until
				// it needs to start moving forward again.  These
				// essentially are gaps in the structure of the activity.
//				if (props.size() == 0) {
//					continue;
//				}

				BPPNode node = new BPPNode(propList, state, startNode);
				Edge e = (Edge) graph.findEdge(lastNode, node);
				if (e == null) {
					e = new Edge(props);
					graph.addEdge(e, lastNode, node);
				}
				e.increment();

				lastNode = node;
			}

		}
		
		initDistanceToNearestFinal(graph);

		return graph;
	}
	
	/**
	 * Set the distance to nearest final state for each node in the FSM.
	 * @param fsm The FSM graph to manipulate.
	 */
	public static void initDistanceToNearestFinal(DirectedGraph<BPPNode, Edge> fsm) {
		
		// Reset distance to max int for all nodes
		for (BPPNode n : fsm.getVertices()) {
			n.setDistanceToFinal(Integer.MAX_VALUE);
		}
		
		Queue<BPPNode> queue = new LinkedList<BPPNode>();
		
		// Init queue with states that have no out edges
		for (BPPNode n : fsm.getVertices()) {
			if (fsm.getOutEdges(n).size() == 0) {
				n.setDistanceToFinal(0);
				queue.add(n);
			}
		}
		
		// Go backward and increment 1 for each step
		BPPNode dest;
		while (!queue.isEmpty()) {
			dest = queue.poll();
			for (Edge inEdge : fsm.getInEdges(dest)) {
				BPPNode source = fsm.getSource(inEdge);
				if (source.isFinal() && source.getDistanceToFinal() != 0) {
					source.setDistanceToFinal(0);
					if (!queue.contains(source))
						queue.add(source);
				} else if (source.getDistanceToFinal() > dest.getDistanceToFinal() + 1){
					source.setDistanceToFinal(dest.getDistanceToFinal() + 1);
					if (!queue.contains(source))
						queue.add(source);
				}
			}
		}
	}

	public static void toDot(DirectedGraph<BPPNode, Edge> graph, String file) {
		toDot(graph, file, true);
	}

	public static void toDot(DirectedGraph<BPPNode, Edge> graph, String file,
			boolean edgeProb) {
		try {
			BufferedWriter out = new BufferedWriter(new FileWriter(file));
			out.write("digraph G { \n");
			out.write("\tgraph [ rankdir=LR ]; \n");

			for (BPPNode vertex : graph.getVertices()) {
				out.write(vertex.toDot());

				double total = 0.0D;
				for (Edge e : graph.getOutEdges(vertex)) {
					total += e.count();
				}

				for (Edge e : graph.getOutEdges(vertex)) {
					BPPNode end = (BPPNode) graph.getDest(e);
					double prob = e.count() / total;

					out.write("\t\"" + vertex.id() + "\" -> \"" + end.id()
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