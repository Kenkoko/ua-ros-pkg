package edu.arizona.cs.learn.timeseries.clustering.kmeans;

import edu.arizona.cs.learn.algorithm.alignment.GeneralAlignment;
import edu.arizona.cs.learn.algorithm.alignment.Params;
import edu.arizona.cs.learn.algorithm.alignment.Report;
import edu.arizona.cs.learn.algorithm.alignment.Similarity;
import edu.arizona.cs.learn.timeseries.distance.Distances;
import edu.arizona.cs.learn.timeseries.model.Instance;

public class InstanceCluster extends Cluster {
	
	private Instance _mediod;

	public InstanceCluster(int id) {
		super(id);
	}
	
	public InstanceCluster(String name, int id) { 
		super(name, id);
	}

	@Override
	public void computeCentroid() {
		// first compute all of the distances from one instance to another and construct
		// a matrix of them.
		double[][] distances = Distances.distances(_instances);
		
		// now compute the sum distance from each point to each other point and
		// save the lowest distance
		double min = Double.MAX_VALUE;
		Instance keep = null;
		for (int i = 0; i < _instances.size(); ++i) { 
			double total = 0;
			for (int j = 0; j < _instances.size(); ++j) { 
				if (i == j)
					continue;
				
				total += distances[i][j];
			}
			
			if (total < min) { 
				min = total;
				keep = _instances.get(i);
			}
		}
		
		_mediod = keep;
	}

	@Override
	public double distance(Instance instance) {
		if (_instances.size() == 0) {
			return Math.random();
		}
		
		Params p = new Params(_mediod.sequence(), instance.sequence());
		p.setMin(0,0);
		p.setBonus(1, 1);
		p.setPenalty(-1, -1);
		p.normalize = normalize;
		p.similarity = Similarity.strings;
		
		Report report = GeneralAlignment.align(p);
		if (Double.compare(report.score, Double.NaN) == 0) { 
			System.out.println("WTF?: " + _mediod.sequence().size() + " -- " + instance.sequence().size());
		}
		return report.score;
	}

}
