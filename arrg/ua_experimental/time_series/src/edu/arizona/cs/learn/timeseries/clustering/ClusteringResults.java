package edu.arizona.cs.learn.timeseries.clustering;

import java.util.List;
import java.util.Map;
import java.util.TreeMap;

import edu.arizona.cs.learn.timeseries.clustering.kmeans.Cluster;
import edu.arizona.cs.learn.timeseries.model.Instance;

public class ClusteringResults {

	public List<Cluster> groundTruth;
	public List<Cluster> clusters;
	
	public int tp;
	public int fp;
	public int fn;
	public int tn;
	
	public Map<String,Double> performance;
	public double accuracy;
	
	public ClusteringResults(List<Cluster> groundTruth, List<Cluster> clusters) { 
		this.groundTruth = groundTruth;
		this.clusters = clusters;
		
		performance();
	}
	
	/**
	 * Compute the performance of the algorithm with regards to ground truth.
	 * @return
	 */
	public void performance() { 
		performance = new TreeMap<String,Double>();
		
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
			
			// TODO: record the id of the closest since it might
			// help us with something.
			performance.put(gt.name(), max);
			total += max;
		}
		
		double average = total / (double) groundTruth.size();
		accuracy = average;
	}
	
	public void accordance(List<Instance> instances) { 
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
					++tp;
				else if (k && !f)
					++fn;
				else if (!k && f)
					++fp;
				else if (!k && !f)
					++tn;
			}
		}

//		out.println("Accordance Ratio 1 (n1) " + (n1 / (n1+n2)));
//		out.println("Accordance Ratio 2 (n4) " + (n4 / (n3+n4)));
	}
}
