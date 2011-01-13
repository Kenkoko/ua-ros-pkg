package edu.arizona.cs.learn.timeseries.clustering.kmeans;

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
	public void cluster(final List<Instance> instances, ClusterInit init, int seedAmt) { 
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
			instances.get(i).uniqueId(i+1000);
		}
		
		final List<Cluster> clusters = new ArrayList<Cluster>();
		for (int i = 0; i < _k; ++i) 
			clusters.add(new Cluster(i));
		
		// Initialize the first clusters according to the method given.
		init.pickCenters(clusters, instances, seedAmt);
		
		int iteration = 1;
		boolean changing = true;
		while (changing && iteration <= _maxIter) {
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
					if (newId == null)
						changing = true;
					else if (dc.clusterId != newId)
						changing = true;
				} catch (Exception e) { 
					e.printStackTrace();
				}
			}

			clusters.clear();
			clusters.addAll(tmp);
			++iteration;
		}
		
		for (Cluster c : clusters) { 
			System.out.println("Cluster: " + c.id());
			System.out.print("\t[");
			for (Instance instance : c.instances()) { 
				System.out.print(instance.name()+ "-" + instance.id() + ",");
			}
			System.out.println("]");
		}
		
		// now we can measure performance
		System.out.println("Performance: " + performance(groundTruth, clusters));
		
		_execute.shutdown();
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
	
	public static double performance(List<Cluster> groundTruth, List<Cluster> clusters) { 
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
			
			System.out.println("...Cluster " + gt.name() + " -- closest: " + closest.id() + " -- " + max);
			total += max;
			
		}
		return total / (double) groundTruth.size();
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

//		for (int i = 0; i < 10; ++i) { 
//			all.add(set1.get(i));
//			all.add(set2.get(i));
//			all.add(set3.get(i));
//			all.add(set4.get(i));
//			all.add(set5.get(i));
////			all.add(set6.get(i));
//		}
		
		all.addAll(set1);
		all.addAll(set2);
		all.addAll(set3);
		all.addAll(set4);
		all.addAll(set5);
//		all.addAll(set6);

		KMeans kmeans = new KMeans(5, 20);
		kmeans.cluster(all, ClusterInit.supervised, 5);
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

