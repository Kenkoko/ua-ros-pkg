package edu.arizona.cs.learn.timeseries.clustering.kmeans;

import java.io.PrintStream;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.TreeMap;
import java.util.concurrent.Callable;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;

import edu.arizona.cs.learn.timeseries.model.Instance;
import edu.arizona.cs.learn.timeseries.model.SequenceType;
import edu.arizona.cs.learn.util.Utils;

public class KMeans {

	private ExecutorService _execute;

	private int _k;
	private int _maxIter;
	
	public KMeans(int k, int maxIter) { 
		_k = k;
		_maxIter = maxIter;
	}
	
	/**
	 * Use the KMeans to perform the clustering and the KMeans++ algorithm
	 * to select initial clusters.
	 * @param instances
	 */
	public void cluster(PrintStream out, final List<Instance> instances, ClusterInit init, int seedAmt) { 
		_execute = Executors.newFixedThreadPool(Utils.numThreads);

		// Make ground truth clusters....
		Map<String,Cluster> gtClusters = new HashMap<String,Cluster>();
		for (Instance instance : instances) { 
			Cluster c = gtClusters.get(instance.name());
			if (c == null) { 
				c = new Cluster(instance.name(), gtClusters.size());
				gtClusters.put(instance.name(), c);
			}
			c.add(instance);
		}
		List<Cluster> groundTruth = new ArrayList<Cluster>(gtClusters.values());
		
		
		// assign uniqueIds to each of the instances....
		for (int i = 0; i < instances.size(); ++i) { 
			instances.get(i).uniqueId(i+100);
		}
		
		final List<Cluster> clusters = new ArrayList<Cluster>();
		for (int i = 0; i < _k; ++i) 
			clusters.add(new Cluster(i));
		
		// Initialize the first clusters according to the method given.
		init.pickCenters(clusters, instances, seedAmt);
		
		out.println("Initialization");
		printClusters(out, clusters);
		
		int iteration = 1;
		boolean changing = true;
		while (changing && iteration <= _maxIter) {
			changing = false;
			
//			if (iteration % 10 == 0) 
			System.out.println("Iteration: " + iteration);

			finishClusters(clusters);
			
			// Fill the map containing the current mapping of instances to clusters
			final Map<Integer,Integer> clusterMap = new TreeMap<Integer,Integer>();
			for (Cluster c : clusters) { 
				for (Instance instance : c.instances())
					clusterMap.put(instance.uniqueId(), c.id());
			}

			// Initialize the new clusters.
			List<Cluster> tmp = new ArrayList<Cluster>();
			for (int i = 0; i < _k; ++i) 
				tmp.add(new Cluster(i));
			
			List<Future<ClusterDistance>> futureList = new ArrayList<Future<ClusterDistance>>();
			for (int i = 0; i < instances.size(); ++i) 
				futureList.add(_execute.submit(new ClusterDistance(instances.get(i), clusters)));
			
			for (Future<ClusterDistance> future : futureList) { 
				try { 
					ClusterDistance dc = future.get();
					tmp.get(dc.clusterId).add(dc.instance);
					
					Integer newId = clusterMap.get(dc.instance.uniqueId());
					if (newId == null) {
						changing = true;
						System.out.println("\t\tNew Assignment " + dc.instance.uniqueId());
					} else if (dc.clusterId != newId) {
						changing = true;
						System.out.println("\t\tChanged cluster: " + dc.instance.uniqueId() + " -- " + dc.clusterId + " -- " + newId);
					}
				} catch (Exception e) { 
					e.printStackTrace();
				}
			}

			clusters.clear();
			clusters.addAll(tmp);
			printClusters(out, clusters);
			
			++iteration;
		}
		
		printClusters(out, clusters);
		
		// now we can measure performance
		performance(out, groundTruth, clusters);
		oatesPerformance(out, instances, groundTruth, clusters);

		_execute.shutdown();
	}
	
	public void printClusters(PrintStream out, List<Cluster> clusters) {
		out.println("Clusters...");
		for (Cluster c : clusters) { 
			out.println("  Cluster: " + c.id());
			out.print("  \t[");
			for (Instance instance : c.instances()) { 
				out.print(instance.name()+ "-" + instance.id() + ",");
			}
			out.println("]");
		}
	}
	
	/**
	 * Iterate over all of the clusters and call finish on them so 
	 * that we can build the correct signature.
	 * @param clusters
	 */
	public void finishClusters(List<Cluster> clusters) { 
		// Call finish on all of the clusters....
		List<Future<ClusterFinish>> finishList = new ArrayList<Future<ClusterFinish>>();
		for (Cluster c : clusters) 
			finishList.add(_execute.submit(new ClusterFinish(c)));
		
		for (Future<ClusterFinish> future : finishList) {
			try { 
				ClusterFinish finish = future.get();
			} catch (Exception e) { 
				e.printStackTrace();
			}
		}
	}
	
	
	public static void oatesPerformance(PrintStream out, List<Instance> instances, List<Cluster> groundTruth, List<Cluster> clusters) { 
		double n1 = 0;
		double n2 = 0;
		double n3 = 0;
		double n4 = 0;
		
		for (int i = 0; i < instances.size(); ++i) { 
			// figure out which cluster this instance is part of in the
			// ground truth as well as the found clusters
			Instance i1 = instances.get(i);
			int true1 = -1;
			for (Cluster c : groundTruth) {
				if (c.contains(i1))
					true1 = c.id();
			}
			
			int found1 = -1;
			for (Cluster c : clusters) { 
				if (c.contains(i1))
					found1 = c.id();
			}
			
			for (int j = i+1; j < instances.size(); ++j) { 
				Instance i2 = instances.get(j);
				int true2 = -1;
				for (Cluster c : groundTruth) {
					if (c.contains(i2))
						true2 = c.id();
				}
				
				int found2 = -1;
				for (Cluster c : clusters) {
					if (c.contains(i2))
						found2 = c.id();
				}
				
				// Now classify... 
				boolean k = true1 == true2;
				boolean f = found1 == found2;
				
				if (k && f) 
					++n1;
				else if (k && !f)
					++n3;
				else if (!k && f)
					++n2;
				else if (!k && !f)
					++n4;
			}
		}
		
		// print out things for the time being, since I'm unsure if I want to dump it to a file
		out.println("Oates table:");
		out.println("---- " + n1 + "\t" + n2);
		out.println("---- " + n3 + "\t" + n4);

		out.println("Accordance Ratio 1 (n1) " + (n1 / (n1+n2)));
		out.println("Accordance Ratio 2 (n4) " + (n4 / (n3+n4)));
	}
	
	public static double performance(PrintStream out, List<Cluster> groundTruth, List<Cluster> clusters) { 
		double total = 0;
		for (Cluster gt : groundTruth) { 
			// find the cluster that maximizes the overlap with this one
			double max = 0;
			Cluster closest = null;
			
			for (Cluster c : clusters) {
				double d = gt.sim(c);
				if (d > max) { 
					max = d;
					closest = c;
				}
			}
			
			out.println("...Cluster " + gt.name() + " -- closest: " + closest.id() + " -- " + max);
			total += max;
			
		}
		
		double average = total / (double) groundTruth.size();
		out.println("Average Performance: " + average);
		return average;
	}
	
	public static void main(String[] args) { 
		// perform a test with a small subset of the total problem.
		List<Instance> all = new ArrayList<Instance>();
		List<Instance> set1 = Utils.sequences("A", "data/input/ww3d-jump-over.lisp", SequenceType.allen);
		List<Instance> set2 = Utils.sequences("B", "data/input/ww3d-jump-on.lisp", SequenceType.allen);
		List<Instance> set3 = Utils.sequences("C", "data/input/ww3d-left.lisp", SequenceType.allen);
		List<Instance> set4 = Utils.sequences("D", "data/input/ww3d-right.lisp", SequenceType.allen);
		List<Instance> set5 = Utils.sequences("E", "data/input/ww3d-push.lisp", SequenceType.allen);
//		List<Instance> set6 = Utils.sequences("F", "data/input/ww3d-approach.lisp", SequenceType.allen);

		for (int i = 0; i < 10; ++i) { 
			all.add(set1.get(i));
			all.add(set2.get(i));
			all.add(set3.get(i));
			all.add(set4.get(i));
			all.add(set5.get(i));
//			all.add(set6.get(i));
		}
		
//		all.addAll(set1);
//		all.addAll(set2);
//		all.addAll(set3);
//		all.addAll(set4);
//		all.addAll(set5);
////		all.addAll(set6);

		KMeans kmeans = new KMeans(5, 20);
		kmeans.cluster(System.out, all, ClusterInit.supervised, 5);
	}

	
}

/**
 * Iterate over all of the clusters and find the one with 
 * the smallest distance to us.
 * @author wkerr
 *
 */
class ClusterDistance implements Callable<ClusterDistance> {
	private List<Cluster> _clusters;
	
	public Instance instance;
	public int clusterId;

	public boolean changing = false;
	
	public ClusterDistance(Instance instance, List<Cluster> clusters) {
		this.instance = instance;

		_clusters = clusters;
	}

	@Override
	public ClusterDistance call() throws Exception {
		double minD = Double.POSITIVE_INFINITY;
		clusterId = -1;
		for (Cluster c : _clusters) { 
			double d = c.distance(instance);
			if (d < minD) { 
				minD = d;
				clusterId = c.id();
			}
		}
		if (clusterId == -1)
			throw new RuntimeException("Impossible: ---" + _clusters.size());
		return this;
	} 
}

class ClusterFinish implements Callable<ClusterFinish> {
	private Cluster _cluster;
	
	public ClusterFinish(Cluster c) { 
		_cluster = c;
	}

	@Override
	public ClusterFinish call() throws Exception {
		_cluster.finish();
		return this;
	}
	
}

