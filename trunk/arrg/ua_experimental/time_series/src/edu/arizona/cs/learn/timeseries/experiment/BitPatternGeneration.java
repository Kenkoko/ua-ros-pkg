package edu.arizona.cs.learn.timeseries.experiment;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.TreeSet;

import org.apache.commons.collections15.Bag;
import org.apache.commons.collections15.bag.TreeBag;
import org.apache.log4j.Logger;

import edu.arizona.cs.learn.algorithm.bpp.BPPFactory;
import edu.arizona.cs.learn.algorithm.markov.BPPNode;
import edu.arizona.cs.learn.algorithm.markov.FSMFactory;
import edu.arizona.cs.learn.algorithm.render.Paint;
import edu.arizona.cs.learn.timeseries.model.Instance;
import edu.arizona.cs.learn.timeseries.model.Interval;
import edu.arizona.cs.learn.timeseries.model.SequenceType;
import edu.arizona.cs.learn.timeseries.model.Signature;
import edu.arizona.cs.learn.timeseries.model.symbols.StringSymbol;
import edu.arizona.cs.learn.timeseries.model.symbols.Symbol;
import edu.arizona.cs.learn.timeseries.visualization.TableFactory;
import edu.arizona.cs.learn.util.Utils;
import edu.arizona.cs.learn.util.graph.Edge;
import edu.uci.ics.jung.graph.DirectedGraph;

public class BitPatternGeneration {
    private static Logger logger = Logger.getLogger(BitPatternGeneration.class);
    
    /**
     * Train the signature and then extract the alignment table that only contains
     * those relations that have been seen at least min times.
     * @param name
     * @param generator
     * @param min
     * @return
     */
    public static List<Symbol[]> trainAndSubset(String name, SequenceType type, int min) { 
    	List<Instance> instances = Utils.sequences(name, "data/input/" + name + ".lisp", type);
		Signature signature = new Signature(name);
		signature.train(instances);
		
		// after training the table still looks good from what I can tell.
		logger.debug("Finished training, beginning validation");
		TableFactory.validateTable(signature.table());
		logger.debug("\tvalidated: " + signature.table().size());

		List<Symbol[]> subset = TableFactory.subset(signature, min);

		logger.debug("Constructed subset, beginning validation");
		TableFactory.validateTable(subset);
		logger.debug("\tvalidated: " + subset.size());
		
		return subset;
    }
    
    /**
     * Genererate the bit patterns for all of the training instances
     * seen after training.
     * @param name
     * @param generator
     * @param min
     * @return
     */
    public static List<List<Interval>> getBPPs(String name, List<Symbol[]> subset, Set<String> propSet) { 
		
		// I need to construct a sorted union of all of the possible propositions
		// within the subset.
		List<List<Interval>> bppInstances = new ArrayList<List<Interval>>();
		
		// each row in the table is a single bit pattern....
		for (int i = 0; i < subset.size(); ++i) { 
			Symbol[] row = subset.get(i);
			Map<String,Interval> map = new HashMap<String,Interval>();
			Bag<String> b = new TreeBag<String>();
			
			// each object is actually an AllenRelation if we used the right generator
			// therefore we can just grab the two intervals and add them
			// to the list.
			for (Symbol obj : row) { 
				if (obj == null)
					continue;
				
				StringSymbol ss = (StringSymbol) obj;
				for (Interval interval : ss.getIntervals()) {
					if (!map.containsKey(interval.toString()))  
						map.put(interval.toString(), interval);
					
					b.add(interval.toString());
					propSet.add(interval.name);
				}
			}
			
			if (map.size() == 0) 
				continue;
			
			// I want to determine how many times each of the intervals were seen.
//			logger.debug("Number of intervals: " + map.size());
//			List<String> removeList = new ArrayList<String>();
//			for (String s : b.uniqueSet()) { 
//				logger.debug("\t" + s + " - " + b.getCount(s));
//				// if an interval participates in less than 1/3 of the total number of relations
//				// that it could participate in, it can be pruned....
//				if (b.getCount(s) < ((float) map.size()) * 0.3333) { 
//					logger.debug("\t\tremoving: " + s);
//					removeList.add(s);
//				}
//			}
//			
//			for (String s : removeList) { 
//				map.remove(s);
//			}
			
			
			List<Interval> intervals = new ArrayList<Interval>(map.values());
			List<Interval> bpp = BPPFactory.compress(intervals, Interval.eff);

			// make two renderings... bpp and original.
//			Paint.render(intervals, "data/images/original/" + name + "-" + i + ".png");
//			Paint.render(bpp, "data/images/bpp/" + name + "-" + i + ".png");
			
			bppInstances.add(bpp);
		}
    	return bppInstances;
    }
    
    /**
     * Genererate the bit patterns for all of the training instances
     * seen after training.
     * @param name
     * @param generator
     * @param min
     * @return
     */
    public static List<List<Interval>> getBPPs(String name, SequenceType type, int min, Set<String> propSet) { 
		List<Symbol[]> subset = trainAndSubset(name, type, min);
		return getBPPs(name, subset, propSet);
    }
    
	public static void generate(String name, SequenceType type, int min, boolean onlyStart) { 
    	List<Instance> instances = Utils.sequences(name, "data/input/" + name + ".lisp", type);
		Signature signature = new Signature(name);
		signature.train(instances);
		
		// after training the table still looks good from what I can tell.
		logger.debug("Finished training, beginning validation");
		TableFactory.validateTable(signature.table());
		logger.debug("\tvalidated: " + signature.table().size());

		List<Symbol[]> subset = TableFactory.subset(signature, min);

		logger.debug("Constructed subset, beginning validation");
		TableFactory.validateTable(subset);
		logger.debug("\tvalidated: " + subset.size());
		
		// I need to construct a sorted union of all of the possible propositions
		// within the subset.
		Set<String> propSet = new TreeSet<String>();
		List<List<Interval>> bppInstances = getBPPs(name, type, min, propSet);

		logger.debug("Num constructed: " + bppInstances.size() + " " + propSet);
		if (bppInstances.size() > 0) { 
			List<String> propList = new ArrayList<String>(propSet);
			DirectedGraph<BPPNode,Edge> graph = FSMFactory.makeGraph(propList, bppInstances, onlyStart);
			FSMFactory.toDot(graph, name + "-" + min + "-bpp.dot");
		} else { 
			logger.debug("Pruning too strongly: " + name + " " + min);
		}
		
	}
	
	public static void compositeGraph(String[] names, int[] mins, SequenceType type) { 
		Set<String> propSet = new TreeSet<String>();
		List<List<List<Interval>>> list = new ArrayList<List<List<Interval>>>();
		
		for (int i = 0; i < names.length; ++i) { 
			list.add(getBPPs(names[i], type, mins[i], propSet));
		}
		
		List<String> propList = new ArrayList<String>(propSet);
		DirectedGraph<BPPNode,Edge> graph = null;
		Map<String,BPPNode> vertices = new HashMap<String,BPPNode>();
		for (int i = 0; i < names.length; ++i) { 
			graph = BPPFactory.graph(graph, vertices, names[i], propList, list.get(i));
		}
		
		FSMFactory.toDot(graph, "aggregate.dot");
	}
	
	public static void supergraph(String[] names, int[] mins, SequenceType type, boolean onlyStart) { 
		Signature signature = new Signature("super");
		for (int i = 0; i < names.length; ++i) { 
			String name = names[i];
	    	List<Instance> instances = Utils.sequences(name, "data/input/" + name + ".lisp", type);
			Signature s = new Signature(name);
			s.train(instances);

			Signature pruned = s.prune(mins[i]);
			signature.merge(pruned);
		}
		
		List<Symbol[]> subset = TableFactory.subset(signature, 0);
		Set<String> propSet = new TreeSet<String>();
		List<List<Interval>> bppInstances = getBPPs("super", subset, propSet);
		if (bppInstances.size() > 0) { 
			List<String> propList = new ArrayList<String>(propSet);
			DirectedGraph<BPPNode,Edge> graph = FSMFactory.makeGraph(propList, bppInstances, onlyStart);
			FSMFactory.toDot(graph, "super.dot");
		} else { 
			logger.debug("Pruning too strongly super");
		}
	}
	
	public static void main(String[] args) { 
		
//    	SequenceGenerator generator = SequenceFactory.sortedAllenGenerator();
    	
//    	String[] names = new String[] { "ww-jump-over", "ww-jump-on", "wubble-jump" };
//    	int[] mins = new int[] { 35, 18, 34 };
//    	supergraph(names, mins, generator);
		
		generate("chpt1-approach", SequenceType.starts, 2,false);

//    	String[] names = new String[] { "ww-jump-over", "ww-approach", "wubble-jump" };
//    	int[] mins = new int[] { 36, 20, 34 };
//    	compositeGraph(names, mins, generator);
    	
//    	generate("ww-jump-over", generator, 36);
//    	generate("ww-jump-over", generator, 35);
//    	generate("ww-jump-over", generator, 30);
//
//    	generate("ww-approach", generator, 20);
//    	generate("ww-approach", generator, 19);
//    	generate("ww-approach", generator, 18);

//    	generate("wubble-jump", generator, 34);
//    	generate("wubble-jump", generator, 33);
//    	generate("wubble-jump", generator, 32);
    	
//    	generate("ww-approach", generator, 15);
//    	generate("wubble-jump", generator, 25);
//    	generate("wubble-run-jump", generator, 20);
//    	generate("wes-pen-r", generator, 18);
	}
}
