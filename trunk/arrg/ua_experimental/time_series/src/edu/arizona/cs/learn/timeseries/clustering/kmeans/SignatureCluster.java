package edu.arizona.cs.learn.timeseries.clustering.kmeans;

import edu.arizona.cs.learn.algorithm.alignment.GeneralAlignment;
import edu.arizona.cs.learn.algorithm.alignment.Normalize;
import edu.arizona.cs.learn.algorithm.alignment.Params;
import edu.arizona.cs.learn.algorithm.alignment.Report;
import edu.arizona.cs.learn.algorithm.alignment.Similarity;
import edu.arizona.cs.learn.timeseries.model.Instance;
import edu.arizona.cs.learn.timeseries.model.Signature;

public class SignatureCluster extends Cluster {

	private Signature _signature;
	
	public SignatureCluster(int id) { 
		super(id);
	}
	
	public SignatureCluster(String name, int id) { 
		super(name, id);
	}
	
	public void clear() { 
		super.clear();
		_signature = new Signature(id() + "");
	}
	
	public void computeCentroid() { 
		if (_instances.size() == 0) {
			System.out.println("[finish] Shit --- a cluster is empty");
			return;
		}
		
		for (int i = 1; i <= _instances.size(); ++i) { 
			if (i % 10 == 0) 
				_signature = _signature.prune(3);
			_signature.update(_instances.get(i-1).sequence());
		}

		System.out.println("Finished: " + _signature.signature().size());
//		int min = (int) Math.floor(0.5 * (double) _indexes.size());
//		_signature = _signature.prune(min);
	}	
	
	/**
	 * Return the distance between this signature for this
	 * cluster and the given instance.
	 * @param instance
	 * @return
	 */
	public double distance(Instance instance) { 
		if (_instances.size() == 0) {
			return Math.random();
		}
		
		Params p = new Params(_signature.signature(), instance.sequence());
		p.setMin(0,0);
		p.setBonus(1, 1);
		p.setPenalty(-1, -1);
		p.normalize = normalize;
		p.similarity = Similarity.strings;
		
		Report report = GeneralAlignment.align(p);
		if (Double.compare(report.score, Double.NaN) == 0) { 
			System.out.println("WTF?: " + _signature.signature().size() + " -- " + instance.sequence().size());
		}
		return report.score;
	}
}
