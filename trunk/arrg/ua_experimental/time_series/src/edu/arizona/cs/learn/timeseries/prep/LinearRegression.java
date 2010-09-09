package edu.arizona.cs.learn.timeseries.prep;

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.util.Arrays;
import java.util.List;
import java.util.Map;
import java.util.TreeMap;

import org.apache.commons.collections15.Transformer;
import org.apache.commons.math.stat.regression.SimpleRegression;

import edu.uci.ics.jung.algorithms.shortestpath.DijkstraShortestPath;
import edu.uci.ics.jung.graph.DirectedGraph;
import edu.uci.ics.jung.graph.DirectedSparseGraph;

public class LinearRegression {

	
	public static List<Edge> fitRegressionLines(List<Double> timeSeries) { 
		// Create a node for each point in the timeSeries
		DirectedGraph<Integer,Edge> graph = new DirectedSparseGraph<Integer,Edge>(); 
		for (int i = 0; i < timeSeries.size(); ++i) {
			graph.addVertex(i);
		}
		
		for (int i = 0; i < timeSeries.size()-1; ++i) { 
			// if this value == NaN then add a zero weight edge to the graph
			if (Double.compare(timeSeries.get(i), Double.NaN) == 0) {
				graph.addEdge(new Edge(i, i+1, 0.0), i, i+1);
				continue;
			}
			
			// if the next value == NaN then add a zero weight edge to the graph
			if (Double.compare(timeSeries.get(i+1), Double.NaN) == 0) {
				graph.addEdge(new Edge(i, i+1, 0.0), i, i+1);
				continue;
			}
						
			for (int j = i + 2; j < timeSeries.size(); ++j) { 
				if (Double.compare(timeSeries.get(j), Double.NaN) == 0) 
					break;
				
				Edge e = new Edge(i, j, timeSeries);
				graph.addEdge(e, i, j);
			}
		}

//		for (int i = 0; i < timeSeries.size(); ++i) { 
//			System.out.println("Node " + i);
//			Map<Integer,Double> weightMap = new TreeMap<Integer,Double>();
//			for (Edge e : graph.getOutEdges(i)) { 
//				weightMap.put(e.endIndex, e.weight);
//			}
//			
//			for (Integer key : weightMap.keySet()) { 
//				System.out.println(" -- " + key + "   weight: " + weightMap.get(key));
//			}
//		}
//		
		Transformer<Edge,Double> t = new Transformer<Edge,Double>() {
			@Override
			public Double transform(Edge edge) {
				return edge.weight;
			} 
		};
		
		DijkstraShortestPath<Integer,Edge> path = new DijkstraShortestPath<Integer,Edge>(graph, t);
		List<Edge> edges = path.getPath(0, timeSeries.size()-1);

		return edges;
	}
	
	public static void write(List<Edge> edges) { 
		try { 
			BufferedWriter out = new BufferedWriter(new FileWriter("/tmp/test.dat"));

			out.write("x,y\n");

			for (Edge edge : edges) { 
				if (edge.isNaN()) { 
					continue;
				}

				out.write(edge.startIndex + "," + edge.regression.predict(edge.startIndex) + "\n");
				out.write(edge.endIndex + "," + edge.regression.predict(edge.endIndex) + "\n");
			}
			out.close();
		} catch (Exception e) { 
			e.printStackTrace();
		}

	}
	
	public static void main(String[] args) { 
		
		Map<String,List<Double>> map = TimeSeries.load("data/raw-data/trace/c03_2.dat", false, "[ ]");
		String variable = "var3";
		
		List<Double> column = map.get(variable);
		
		column = TimeSeries.linearFilter(column, 10);
		column = TimeSeries.standardize(column);
		
		// write out the column...
		try { 
			BufferedWriter out = new BufferedWriter(new FileWriter("/tmp/column.dat"));
			out.write("x,y\n");
			for (int i = 0; i < column.size(); ++i) { 
				out.write(i + "," + column.get(i) + "\n");
			}
			out.close();
		} catch (Exception e) { 
			e.printStackTrace();
		}
		
		List<Edge> edges = fitRegressionLines(column);
		write(edges);

		List<Double> breakpoints = Arrays.asList(-2.0, -0.5, 0.5, 2.0);
		List<String> classes = Arrays.asList("steep-down", "down", "stable", "up", "steep-up");
		
		List<String> symbols = TimeSeries.regression(column, breakpoints, classes);
		System.out.println("Size: " + column.size() + " " + symbols.size());
		try { 
			BufferedWriter out = new BufferedWriter(new FileWriter("/tmp/symbols.dat"));
			out.write("x,y\n");
			for (int i = 0; i < symbols.size(); ++i) { 
				String symbol = symbols.get(i);
				if ("NaN".equals(symbol)) { 
					out.write(i + ",-2\n");
				} else if ("down".equals(symbol)) { 
					out.write(i + ",-1\n");
				} else if ("up".equals(symbol)) { 
					out.write(i + ",1\n");
				} else if ("stable".equals(symbol)) { 
					out.write(i + ",0\n");
				}
			}
			out.close();
		} catch (Exception e) { 
			e.printStackTrace();
		}
	}
}

class Edge { 
	public int startIndex;
	public int endIndex;
	
	public SimpleRegression regression;
	public double weight;
	
	private boolean _isNaN;
	
	public Edge(int startIndex, int endIndex, double weight) { 
		_isNaN = true;

		this.startIndex = startIndex;
		this.endIndex = endIndex;
		
		this.weight = weight;
	}
	
	public Edge(int startIndex, int endIndex, List<Double> timeSeries) { 
		this.startIndex = startIndex;
		this.endIndex = endIndex;
		
		regression = new SimpleRegression();
		for (int i = startIndex; i <= endIndex; ++i) { 
			regression.addData(i, timeSeries.get(i));
		}
		weight = regression.getMeanSquareError();
	}	
	
	/**
	 * Was this edge constructed because
	 * one of the vertexes were NaN
	 * @return
	 */
	public boolean isNaN() { 
		return _isNaN;
	}
}
