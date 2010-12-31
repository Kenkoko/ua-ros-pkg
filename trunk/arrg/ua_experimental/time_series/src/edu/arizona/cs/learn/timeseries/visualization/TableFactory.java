package edu.arizona.cs.learn.timeseries.visualization;

import java.util.ArrayList;
import java.util.Collection;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

import org.apache.log4j.Logger;

import edu.arizona.cs.learn.timeseries.model.Signature;
import edu.arizona.cs.learn.timeseries.model.symbols.AllenRelation;
import edu.arizona.cs.learn.timeseries.model.symbols.Symbol;
import edu.arizona.cs.learn.timeseries.visualization.graph.Node;
import edu.arizona.cs.learn.util.graph.Edge;
import edu.uci.ics.jung.graph.DirectedGraph;
import edu.uci.ics.jung.graph.DirectedSparseGraph;

public class TableFactory {
	private static Logger logger = Logger.getLogger(TableFactory.class);


	/**
	 * 
	 * @param s
	 * @param min
	 * @return
	 */
	public static List<Symbol[]> subset(Signature s, int min) {
		List<Symbol[]> results = new ArrayList<Symbol[]>(s.table().size());

		Set<Integer> ignore = new HashSet<Integer>();
		for (int i = 0; i < s.signature().size(); i++) {
			Symbol obj = s.signature().get(i);
			if (obj.weight() < min) {
				ignore.add(i);
			}
		}

		int newSize = s.signature().size() - ignore.size();
		logger.debug("Signature size: " + s.signature().size() + " " + ignore.size() + " New size: " + newSize);

		for (int i = 0; i < s.table().size(); i++) {
			Symbol[] newRow = new Symbol[newSize];
			Symbol[] oldRow = s.table().get(i);

			int pos = 0;
			for (int j = 0; j < oldRow.length; j++) {
				if (ignore.contains(Integer.valueOf(j)))
					continue;
				newRow[pos] = oldRow[j];
				pos++;
			}
			results.add(newRow);
		}
		return results;
	}

	/**
	 * 
	 * @param subset
	 * @param s
	 * @return
	 */
	public static DirectedGraph<Node,Edge> buildGraph(List<Symbol[]> subset, Signature s) {
		if (subset.size() == 0) {
			throw new RuntimeException("Subset cannot have length 0");
		}
		int subsetSize = ((Symbol[]) subset.get(0)).length;

		DirectedGraph<Node,Edge> graph = new DirectedSparseGraph<Node,Edge>();
		List<Node> vertexes = new ArrayList<Node>(subsetSize + 2);

		Node start = new Node("start", 0);
		vertexes.add(start);
		graph.addVertex(start);
		for (int i = 0; i < subsetSize; i++) {
			for (Symbol[] row : subset) {
				if (row[i] == null) {
					continue;
				}
				Node n = new Node(row[i], i + 1);
				vertexes.add(n);
				graph.addVertex(n);
				break;
			}
		}
		Node end = new Node("end", subsetSize + 1);
		vertexes.add(end);
		graph.addVertex(end);

		for (Symbol[] row : subset) {
			Node currentState = start;
			for (int i = 0; i < row.length; i++) {
				if (row[i] == null) {
					continue;
				}
				Node nextState = (Node) vertexes.get(i + 1);
				findOrAddEdge(graph, currentState, nextState);
				currentState = nextState;
			}

			Node nextState = (Node) vertexes.get(subsetSize + 1);
			findOrAddEdge(graph, currentState, nextState);
		}
		return graph;
	}

	/**
	 * 
	 * @param graph
	 */
	public static void collapseGraph(DirectedGraph<Node, Edge> graph) {
		int start = 0;
		do {
			start = graph.getVertices().size();
			logger.debug("Starting size: " + start);

			Node removed = null;
			List<Edge> removeEdges = new ArrayList<Edge>();
			for (Node n : graph.getVertices()) {
				Collection<Edge> edges = graph.getOutEdges(n);
				if (edges.size() == 1) {
					Edge edge = (Edge) edges.iterator().next();
					Node dest = (Node) graph.getDest(edge);

					Collection<Edge> destInEdges = graph.getInEdges(dest);
					if (destInEdges.size() > 1) {
						continue;
					}
					logger.debug("\tmerging: " + n.id() + " " + dest.id());
					dest.merge(n);
					removed = n;

					Collection<Edge> inEdges = graph.getInEdges(n);
					for (Edge e : inEdges) {
						Node previous = (Node) graph.getSource(e);
						graph.addEdge(new Edge(e.count()), previous, dest);
					}

					break;
				}
			}

			graph.removeVertex(removed);

			for (Edge e : removeEdges) {
				graph.removeEdge(e);
			}
		} while (start != graph.getVertices().size());
	}

	private static void findOrAddEdge(DirectedGraph<Node, Edge> graph, Node from, Node to) {
		throw new RuntimeException("Need to fix this stuff");
	}

	public static String toLatex(List<Symbol[]> table) {
		StringBuffer buf = new StringBuffer();

		String s = "c|";
		int n = ((Symbol[]) table.get(0)).length;
		String repeated = String.format(
				String.format("%%0%dd", new Object[] { Integer.valueOf(n) }),
				new Object[] { Integer.valueOf(0) }).replaceAll("0", s);
		buf.append("\\begin{tabular}{|l|" + repeated + "}\n\\hline\n");

		int[] totals = new int[n];
		for (int i = 0; i < table.size(); i++) {
			Symbol[] row = (Symbol[]) table.get(i);

			for (int j = 0; j < row.length; j++) {
				if (row[j] == null) {
					buf.append(" &  ");
				} else {
					buf.append(" & " + row[j].latex() + " ");

					totals[j] += 1;
				}
			}
			buf.append(" \\\\ \\hline\n");
		}

		buf.append("\\hline\nTotals ");
		for (int i = 0; i < totals.length; i++) {
			buf.append(" & " + totals[i]);
		}
		buf.append("\\\\ \\hline\n");
		buf.append("\\end{tabular}\n");

		return buf.toString();
	}

	public static void validateTable(List<Symbol[]> table) {
		for (int i = 0; i < table.size(); i++) {
			Symbol[] row = (Symbol[]) table.get(i);
			int rowId = -1;
			for (Symbol obj : row) {
				if (obj == null) {
					continue;
				}
				if (!(obj instanceof AllenRelation)) {
					return;
				}
				AllenRelation relation = (AllenRelation) obj;
				assert (relation.interval1().episode == relation.interval2().episode);

				rowId = relation.interval1().episode;
			}
		}
	}
}
