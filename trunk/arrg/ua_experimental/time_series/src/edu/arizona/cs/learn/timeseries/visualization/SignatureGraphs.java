package edu.arizona.cs.learn.timeseries.visualization;

import java.util.ArrayList;
import java.util.List;

import org.apache.log4j.Logger;

import edu.arizona.cs.learn.timeseries.model.Instance;
import edu.arizona.cs.learn.timeseries.model.SequenceType;
import edu.arizona.cs.learn.timeseries.model.Signature;
import edu.arizona.cs.learn.timeseries.model.symbols.Symbol;
import edu.arizona.cs.learn.timeseries.visualization.graph.GraphMethods;
import edu.arizona.cs.learn.timeseries.visualization.graph.Node;
import edu.arizona.cs.learn.util.Range;
import edu.arizona.cs.learn.util.Utils;
import edu.arizona.cs.learn.util.graph.Edge;
import edu.uci.ics.jung.graph.DirectedGraph;

public class SignatureGraphs {
    private static Logger logger = Logger.getLogger(SignatureGraphs.class);
	
	
	public static void main(String[] args) { 
		Utils.LIMIT_RELATIONS = true;
		Utils.WINDOW = 5;
		
//    	SequenceGenerator generator = SequenceFactory.allenGenerator();
//    	SequenceGenerator generator = SequenceFactory.forwardAllenGenerator();
//    	SequenceGenerator generator = SequenceFactory.propGenerator(Interval.eff);
    	
    	singleGraph("chpt1-approach", SequenceType.starts, 2);
//    	singleGraph("wubble-jump", generator, 30);
//    	singleGraph("ww-jump-over", generator, 30);
//    	List<String> names = new ArrayList<String>();
//    	names.add("wubble-jump");
//    	names.add("wubble-run-jump");
//    	names.add("ww-jump-over");
//    	mergeGraph(names, generator);
	}
	
	public static void singleGraph(String name, SequenceType type, int min) {
    	List<Instance> instances = Utils.sequences(name, "data/input/" + name + ".lisp", type);
		Signature signature = new Signature(name);
		signature.train(instances);

		List<Symbol[]> subset = TableFactory.subset(signature, min);
		// we need to make a set of ranges by walking across the rows.		
		
		DirectedGraph<Node,Edge> graph = TableFactory.buildGraph(subset, signature);
		GraphMethods.toDot(graph, name + "-before.dot");

		TableFactory.collapseGraph(graph);
		GraphMethods.toDot(graph, name + "-after.dot");
	}
	
	public static void mergeGraph(List<String> names, SequenceType type) {
		List<Signature> signatures = new ArrayList<Signature>();
		List<Range> ranges = new ArrayList<Range>();
		
		int min = 0;
		for (String name : names) {  
	    	List<Instance> instances = Utils.sequences(name, "data/input/" + name + ".lisp", type);
			Signature signature = new Signature(name);
			signature.train(instances);
			signatures.add(signature.prune(signature.trainingSize()/2));
			ranges.add(new Range(min, min+signature.trainingSize()));
			
			min += signature.trainingSize();
		}
		
		Signature starter = signatures.get(0);
		for (int i = 1; i < signatures.size(); ++i) { 
			starter.merge(signatures.get(i));
		}
		
		List<Symbol[]> subset = TableFactory.subset(starter, 10);
		DirectedGraph<Node,Edge> graph = TableFactory.buildGraph(subset, starter);
		GraphMethods.color(graph, subset, ranges);
		GraphMethods.toDot(graph, "prop-finish.dot");
	}
	
	public static void subgraphBuilder() { 
//		if (defaultRow[0] instanceof AllenRelation) { 
//			int size = subset.get(0).length;
//			Object[] defaultRow = new Object[size];
//			for (int i = 0; i < size; ++i) { 
//				for (int j = 0; j < subset.size(); ++j) { 
//					Object[] row = subset.get(j);
//					if (row[i] != null) { 
//						defaultRow[i] = row[i];
//						break;
//					}
//				}
//			}
//
//			List<Range> ranges = new ArrayList<Range>();
//			AllenRelation[] allenRow = new AllenRelation[defaultRow.length];
//			for (int i = 0; i < defaultRow.length; ++i) { 
//				allenRow[i] = (AllenRelation) defaultRow[i];
//			}
//			
//			Range current = new Range(0,0);
//			AllenRelation r = allenRow[0];
//			for (int i = 1; i < allenRow.length; ++i) { 
//				if (!r.prop1().equals(allenRow[i].prop1())) { 
//					current.max = i;
//					ranges.add(current);
//					
//					current = new Range(i,0);
//					r = allenRow[i];
//				}
//			}
//			logger.debug("Ranges: " + ranges);
//		}
	}
}
