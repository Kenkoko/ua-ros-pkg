package edu.arizona.cs.learn.timeseries.clustering.kmeans;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Map;
import java.util.Random;
import java.util.TreeMap;
import java.util.concurrent.Callable;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;

import edu.arizona.cs.learn.timeseries.clustering.Distance;
import edu.arizona.cs.learn.timeseries.model.Instance;
import edu.arizona.cs.learn.timeseries.model.SequenceType;
import edu.arizona.cs.learn.util.Utils;

public class KMeans {

	public static void main(String[] args) { 
		// perform a test with a small subset of the total problem.
		List<Instance> all = new ArrayList<Instance>();
		List<Instance> set1 = Utils.sequences("A", "data/input/ww3d-jump-over.lisp", SequenceType.allen);
		List<Instance> set2 = Utils.sequences("B", "data/input/ww3d-jump-on.lisp", SequenceType.allen);
		List<Instance> set3 = Utils.sequences("C", "data/input/ww3d-left.lisp", SequenceType.allen);
		List<Instance> set4 = Utils.sequences("D", "data/input/ww3d-right.lisp", SequenceType.allen);
		List<Instance> set5 = Utils.sequences("E", "data/input/ww3d-push.lisp", SequenceType.allen);
		List<Instance> set6 = Utils.sequences("F", "data/input/ww3d-approach.lisp", SequenceType.allen);

		for (int i = 0; i < 5; ++i) { 
			all.add(set1.get(i));
			all.add(set2.get(i));
//			all.add(set3.get(i));
//			all.add(set4.get(i));
//			all.add(set5.get(i));
//			all.add(set6.get(i));
		}
		
//		all.addAll(set1);
//		all.addAll(set2);
//		all.addAll(set3);
//		all.addAll(set4);
//		all.addAll(set5);
//		all.addAll(set6);

		cluster(2, all, 20);
	}

	/**
	 * Use the KMeans to perform the clustering and the KMeans++ algorithm
	 * to select initial clusters.
	 * @param instances
	 */
	public static void cluster(int k, final List<Instance> instances, int maxIter) { 
		final List<Cluster> clusters = new ArrayList<Cluster>();
		// randomly distribute the clusters
		for (int i = 0; i < k; ++i) 
			clusters.add(new Cluster(i));

		Random r = new Random(System.currentTimeMillis());
		for (int i = 0; i < instances.size(); ++i)  
			clusters.get(r.nextInt(k)).add(i, instances.get(i));
		
		ExecutorService execute = Executors.newFixedThreadPool(Utils.numThreads);
		
		int iteration = 1;
		boolean changing = true;
		while (changing && iteration <= maxIter) {
			if (iteration % 10 == 0) 
				System.out.println("Iteration: " + iteration);

			// Call finish on all of the clusters....
			List<Future<ClusterFinish>> finishList = new ArrayList<Future<ClusterFinish>>();
			for (Cluster c : clusters) 
				finishList.add(execute.submit(new ClusterFinish(c)));
			
			for (Future<ClusterFinish> future : finishList) {
				try { 
					ClusterFinish finish = future.get();
				} catch (Exception e) { 
					e.printStackTrace();
				}
			}
			
			// Fill the map containing the current mapping of instances to clusters
			final Map<Integer,Integer> clusterMap = new TreeMap<Integer,Integer>();
			for (Cluster c : clusters) { 
				for (Integer index : c.indexes())  
					clusterMap.put(index, c.id());
			}

			List<Cluster> tmp = new ArrayList<Cluster>();
			for (int i = 0; i < k; ++i) 
				tmp.add(new Cluster(i));
			
			List<Future<ClusterDistance>> futureList = new ArrayList<Future<ClusterDistance>>();
			for (int i = 0; i < instances.size(); ++i) 
				futureList.add(execute.submit(new ClusterDistance(i, instances.get(i), clusters)));
			
			for (Future<ClusterDistance> future : futureList) { 
				try { 
					ClusterDistance dc = future.get();
					tmp.get(dc.clusterId).add(dc.index, dc.instance);
					if (dc.clusterId != clusterMap.get(dc.index))
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
			for (int i : c.indexes()) { 
				Instance instance = instances.get(i);
				System.out.print(instance.name()+ "-" + instance.id() + ",");
			}
			System.out.println("]");
		}
		
		execute.shutdown();
	}
	
	public static List<Integer> pickCenters(int k, List<Instance> instances) { 
		System.out.print("Calculating distances....");
		System.out.flush();
		double[][] distances = Distance.distancesA(instances);
		System.out.println("done");

		Random r = new Random(System.currentTimeMillis());
		List<Integer> indexes = new ArrayList<Integer>();
		List<Integer> centers = new ArrayList<Integer>(k);

		// 1. Choose one center uniformly at random from among the data points.
		// 2. For each data point x, compute D(x), the distance between x and the nearest center that has already been chosen.
		// 3. Add one new data point at random as a new center, using a weighted probability distribution where a point x is chosen with probability proportional to D(x)2.
		// 4. Repeat Steps 2 and 3 until k centers have been chosen.
		// 5. Now that the initial centers have been chosen, proceed using standard k-means clustering.
		for (int i = 0; i < instances.size(); ++i) 
			indexes.add(i);
		Collections.shuffle(indexes, r);
		centers.add(indexes.remove(0));
		
		while (centers.size() < k) { 
			for (int index : indexes) { 
				double minD = Double.POSITIVE_INFINITY;
				for (int center : centers) { 
					minD = Math.min(distances[index][center], minD);
				}
			}
		}

		throw new RuntimeException("Not yet implemented!");
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
	
	public int index;
	public Instance instance;
	public int clusterId;

	public boolean changing = false;
	
	public ClusterDistance(int index, Instance instance, List<Cluster> clusters) {
		this.index = index;
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

