package edu.arizona.cs.learn.timeseries.clustering.kmeans;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Random;
import java.util.TreeMap;

import edu.arizona.cs.learn.timeseries.clustering.Distance;
import edu.arizona.cs.learn.timeseries.model.Instance;

public enum ClusterInit {

	random {
		@Override
		public void pickCenters(List<Cluster> clusters, List<Instance> instances) {
			Random r = new Random(System.currentTimeMillis());
			LinkedList<Instance> copy = new LinkedList<Instance>(instances);
			Collections.shuffle(copy, r);
			for (int i = 0; i < copy.size(); ++i) { 
				Cluster c = clusters.get(i%clusters.size());
				c.add(copy.get(i));
			}
		}
	},
	
	kPlusPlus {
		@Override
		public void pickCenters(List<Cluster> clusters, List<Instance> instances) {
			System.out.print("Calculating distances....");
			System.out.flush();
			double[][] distances = Distance.distances(instances);
			System.out.println("done");

			Random r = new Random(System.currentTimeMillis());
			List<Integer> indexes = new ArrayList<Integer>();
			List<Integer> centers = new ArrayList<Integer>(clusters.size());

			// 1. Choose one center uniformly at random from among the data points.
			// 2. For each data point x, compute D(x), the distance between x and the nearest center that has already been chosen.
			// 3. Add one new data point at random as a new center, using a weighted probability distribution where a point x is chosen with probability proportional to D(x)2.
			// 4. Repeat Steps 2 and 3 until k centers have been chosen.
			// 5. Now that the initial centers have been chosen, proceed using standard k-means clustering.
			for (int i = 0; i < instances.size(); ++i) 
				indexes.add(i);
			Collections.shuffle(indexes, r);
			centers.add(indexes.remove(0));
			
			while (centers.size() < clusters.size()) { 
				// sum up all of the weights.
				double sum = 0;
				Map<Integer,Double> weight = new TreeMap<Integer,Double>();
				for (int index : indexes) { 
					double minD = Double.POSITIVE_INFINITY;
					for (int center : centers) { 
						minD = Math.min(distances[index][center], minD);
					}
					
					weight.put(index, minD*minD);
					sum += weight.get(index);
				}
				
				int center = -1;
				double random = r.nextDouble();
				double lower = 0;
				for (Integer key : weight.keySet()) { 
					double pct = weight.get(key) / sum;
					double upper = lower + pct;
					if (random < upper) {
						center = key;
						break;
					} else { 
						lower = upper;
					}
				}
				
				centers.add(center);
			}

			for (int i = 0; i < centers.size(); ++i) { 
				clusters.get(i).add(instances.get(centers.get(i)));
			}
		}
	},
	supervised1 {
		@Override
		public void pickCenters(List<Cluster> clusters, List<Instance> instances) {
			supervised(clusters, instances, 1);
		}
	},
	supervised5 {
		@Override
		public void pickCenters(List<Cluster> clusters, List<Instance> instances) { 
			supervised(clusters, instances, 5);
		}
	},
	supervised10 { 
		@Override
		public void pickCenters(List<Cluster> clusters, List<Instance> instances) { 
			supervised(clusters, instances, 10);
		}
	};
	
	/**
	 * Initialize the clusters with potential centers.  
	 * @param clusters
	 * @param instances
	 */
	public abstract void pickCenters(List<Cluster> clusters, List<Instance> instances);
	
	private static void supervised(List<Cluster> clusters, List<Instance> instances, int seedAmt) { 
		// This is a supervised method to initialize the clusters.  First we
		// need to create a mapping from className to List<Instance> instances.
		Random r = new Random(System.currentTimeMillis());
		Map<String,List<Instance>> map = new HashMap<String,List<Instance>>();
		for (Instance instance : instances) { 
			List<Instance> tmp = map.get(instance.name());
			if (tmp == null) { 
				tmp = new ArrayList<Instance>();
				map.put(instance.name(), tmp);
			}
			tmp.add(instance);
		}
		
		if (map.keySet().size() != clusters.size()) 
			throw new RuntimeException("Supervised so the number of clusters should equal the number of classes");
		
		System.out.println("Clusters: " + clusters.size());
		int index = 0;
		for (String className : map.keySet()) { 
			List<Instance> tmp = map.get(className);
			Collections.shuffle(tmp, r);
			
			System.out.println("\tTmp: " + className + " - " + tmp.size());
			for (int i = 0; i < seedAmt; ++i) 
				clusters.get(index).add(tmp.get(i));

			++index;
		}
	}
}

