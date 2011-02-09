package edu.arizona.cs.learn.timeseries.clustering.hierarchical;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Stack;
import java.util.concurrent.Callable;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;

import edu.arizona.cs.learn.timeseries.clustering.Distance;
import edu.arizona.cs.learn.timeseries.model.Instance;
import edu.arizona.cs.learn.timeseries.model.SequenceType;
import edu.arizona.cs.learn.timeseries.model.symbols.Symbol;
import edu.arizona.cs.learn.util.Utils;

public class Agglomerative {
	
	public static void main(String[] args) { 
		// perform a test with a small subset of the total problem.
		List<Instance> all = new ArrayList<Instance>();
		List<Instance> set1 = Utils.sequences("A", "data/input/ww3d-jump-over.lisp", SequenceType.allen);
		List<Instance> set2 = Utils.sequences("B", "data/input/ww3d-jump-on.lisp", SequenceType.allen);
		List<Instance> set3 = Utils.sequences("C", "data/input/ww3d-left.lisp", SequenceType.allen);
		List<Instance> set4 = Utils.sequences("D", "data/input/ww3d-right.lisp", SequenceType.allen);
		List<Instance> set5 = Utils.sequences("E", "data/input/ww3d-push.lisp", SequenceType.allen);
		List<Instance> set6 = Utils.sequences("F", "data/input/ww3d-approach.lisp", SequenceType.allen);

//		for (int i = 0; i < 5; ++i) { 
//			all.add(set1.get(i));
//			all.add(set2.get(i));
//			all.add(set3.get(i));
//			all.add(set4.get(i));
//		}
		
		all.addAll(set1);
		all.addAll(set2);
		all.addAll(set3);
		all.addAll(set4);
		all.addAll(set5);
		all.addAll(set6);

		AgglomerativeNode node = cluster(all, "complete");
		toRScript(node);
	}

	/**
	 * Build up the heirarchical structure and return the top level node
	 * so that the user can decide what to do with it.
	 * @param instances
	 * @param method
	 * @return
	 */
	public static AgglomerativeNode cluster(List<Instance> instances, final String method) { 
		List<List<Symbol>> sequences = new ArrayList<List<Symbol>>(instances.size());
		for (Instance instance : instances)  
			sequences.add(instance.sequence());

		System.out.print("Calculating distances....");
		System.out.flush();
		final double[][] distances = Distance.distances(instances);
		System.out.println("done");
		
		List<AgglomerativeNode> nodes = new ArrayList<AgglomerativeNode>();
		for (int i = 0; i < instances.size(); ++i) { 
			nodes.add(new AgglomerativeNode(i, instances.get(i)));
		}
		
		/** 
		 * Useful class for multithreading the distance calculations.
		 * @author wkerr
		 *
		 */
		class ClusterCallable implements Callable<ClusterCallable> {
			public AgglomerativeNode n1;
			public AgglomerativeNode n2;
			
			public double distance;
			
			public ClusterCallable(AgglomerativeNode n1, AgglomerativeNode n2) { 
				this.n1 = n1;
				this.n2 = n2;
			}
			@Override
			public ClusterCallable call() throws Exception {
				distance = n1.distance(n2, distances, method);
				return this;
			} 
		}
		
		ExecutorService execute = Executors.newFixedThreadPool(Utils.numThreads);
		while (nodes.size() > 1) { 
			AgglomerativeNode minN1 = null;
			AgglomerativeNode minN2 = null;
			double min = Double.POSITIVE_INFINITY;

			List<Future<ClusterCallable>> futureList = new ArrayList<Future<ClusterCallable>>();
			for (int i = 0; i < nodes.size(); i++) {
				AgglomerativeNode n1 = nodes.get(i);
				for (int j = i + 1; j < nodes.size(); j++) {
					AgglomerativeNode n2 = nodes.get(j);
					futureList.add(execute.submit(new ClusterCallable(n1, n2)));
				}
			}
			
			for (Future<ClusterCallable> future : futureList) { 
				try { 
					ClusterCallable dc = future.get();
					if (dc.distance < min) {
						min = dc.distance;
						minN1 = dc.n1;
						minN2 = dc.n2;
					}
				} catch (Exception e) { 
					e.printStackTrace();
				}
			}
			
			nodes.remove(minN1);
			nodes.remove(minN2);

			nodes.add(new AgglomerativeNode(minN1, minN2, min));
		}
		execute.shutdown();
		
		return nodes.get(0);
	}

	public static void toRScript(AgglomerativeNode root) { 
		List<String> labels = new ArrayList<String>();
		Map<AgglomerativeNode,Integer> rowMap = new HashMap<AgglomerativeNode,Integer>();
		
		// we need to traverse the binary tree created by the agglomerative node
		// when we encounter leafs, then we add their label to the list of labels.
		Stack<AgglomerativeNode> frontier = new Stack<AgglomerativeNode>();
		frontier.add(root);
		
		while (!frontier.isEmpty()) { 
			// pop the first... if a leaf add the label.  Otherwise add it's children to the front of the list
			AgglomerativeNode node = frontier.pop();
//			System.out.println("\tNode: " + node.indexes());
			if (node.isLeaf()) { 
				labels.add(node.label());
				rowMap.put(node, -1*labels.size());
				
//				System.out.println("\t\tAdding - " + node.label() + " - " + (-1*labels.size()));
			} else { 
//				System.out.println("\t\tLeft: " + node.left().indexes());
//				System.out.println("\t\tRight: " + node.right().indexes());
				frontier.push(node.right());
				frontier.push(node.left());
			}
		}
		
		List<List<Integer>> rows = new ArrayList<List<Integer>>();
		List<Double> distances = new ArrayList<Double>();
		determineRow(rowMap, rows, distances, root);
		
		StringBuffer buf = new StringBuffer();
		buf.append("a <- list()\n");

		// Add in the row information.
		buf.append("a$merge <- matrix(c(");
		for (List<Integer> row : rows) { 
			buf.append(row.get(0) + "," + row.get(1) + ",");
		}
		buf.deleteCharAt(buf.length()-1);
		buf.append("), nc=2, byrow=TRUE)\n");
		
		// Add in the distance information.
		buf.append("a$height <- c(");
		for (Double d : distances) 
			buf.append(d + ",");
		buf.deleteCharAt(buf.length()-1);
		buf.append(")\n");
		
		buf.append("a$order <- 1:" + labels.size() + "\n");
		buf.append("a$labels <- c(");
		for (String s : labels) 
			buf.append("\"" + s + "\",");
		buf.deleteCharAt(buf.length()-1);
		buf.append(")\n");
		
		buf.append("class(a) <- \"hclust\"\n");
		buf.append("plot(a)\n");
		
		System.out.println(buf);
	}
	
	public static int determineRow(Map<AgglomerativeNode,Integer> map, List<List<Integer>> rows, List<Double> distances, AgglomerativeNode node) { 
		if (node.isLeaf())
			return map.get(node);

		Integer left = map.get(node.left());
		if (left == null) { 
			left = determineRow(map, rows, distances, node.left());
			map.put(node.left(), left);
		}
		
		Integer right = map.get(node.right());
		if (right == null) { 
			right = determineRow(map, rows, distances, node.right());
			map.put(node.right(), right);
		}
		
		distances.add(node.distance());
		rows.add(Arrays.asList(left, right));
		return rows.size();
	}
}
