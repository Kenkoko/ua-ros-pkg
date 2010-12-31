package edu.arizona.cs.learn.timeseries.visualization.graph;

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.util.List;

import edu.arizona.cs.learn.timeseries.model.symbols.Symbol;
import edu.arizona.cs.learn.util.Range;
import edu.arizona.cs.learn.util.Utils;
import edu.arizona.cs.learn.util.graph.Edge;
import edu.uci.ics.jung.graph.DirectedGraph;

public class GraphMethods {
	
	public static void toDot(DirectedGraph<Node,Edge> graph, String file) { 
    	try {
    		BufferedWriter out = new BufferedWriter(new FileWriter(file));
    		out.write("digraph G { \n");
    		out.write("\tgraph [ rankdir=LR ]; \n");
    		
    		// write out all of the vertices and the connections
    		for (Node vertex : graph.getVertices()) { 
    			out.write(vertex.toDot());
    			
    			double total = 0;
        		for (Edge e : graph.getOutEdges(vertex)) { 
        			total += e.count();
        		}
    			
        		for (Edge e : graph.getOutEdges(vertex)) { 
        			Node end = graph.getDest(e);
        			double prob = e.count() / total;
        			
        			out.write("\t\"" + vertex.id() + "\" -> \"" + end.id() + "\" [label=" + Utils.nf.format(prob)  + "];\n");
        		}
    		}
    		out.write("}\n");
    		out.close();
    	} catch (Exception e) { 
    		e.printStackTrace();
    	}
	}
	
    

    /**
     * Color the nodes in the graph by the activity that the Node appears in.
     * For example if the activity appears in all of the ranges, then it will be
     * colored a light gray (white).
     * @param graph
     * @param table
     * @param ranges
     */
    public static void color(DirectedGraph<Node,Edge> graph, List<Symbol[]> table, List<Range> ranges) { 

    	// for each node determine the mixture of classes it appears in.
    	for (Node n : graph.getVertices()) { 
    		if (n.id() == 0 || n.containsKey("end")) { 
    			n.color("lightgray");
    			continue;
    		}
    		
    		StringBuffer buf = new StringBuffer("#");
    		for (Range r : ranges) { 
    			boolean found = false;
    			for (int i = r.min; i < r.max; ++i) { 
    				Object[] row = table.get(i);
    				if (row[n.id()-1] != null) { 
    					found = true;
    					break;
    				}
    			}
    			
    			if (found)
    				buf.append("FF");
    			else
    				buf.append("00");
    		}
    		
    		n.color(buf.toString());
    	}
    }
}
