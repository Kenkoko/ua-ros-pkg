package edu.arizona.cs.learn.timeseries.clustering.kmeans;

import java.util.ArrayList;
import java.util.List;
import java.util.Set;
import java.util.TreeSet;

import edu.arizona.cs.learn.algorithm.alignment.GeneralAlignment;
import edu.arizona.cs.learn.algorithm.alignment.Normalize;
import edu.arizona.cs.learn.algorithm.alignment.Params;
import edu.arizona.cs.learn.algorithm.alignment.Report;
import edu.arizona.cs.learn.algorithm.alignment.Similarity;
import edu.arizona.cs.learn.timeseries.model.Instance;
import edu.arizona.cs.learn.timeseries.model.Signature;

public class Cluster {

	private int _id;
	private List<Instance> _instances;
	
	private Signature _signature;
	
	public Cluster(int id) { 
		_id = id;
		clear();
	}
	
	public int id() { 
		return _id;
	}
	
	public void clear() { 
		_signature = new Signature(_id + "");
		_instances = new ArrayList<Instance>();
	}
	
	public void add(Instance instance) {
		_instances.add(instance);
	}

	public List<Instance> instances() { 
		return _instances;
	}
	
	public void finish() { 
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
		p.setBonus(1, 0);
		p.setPenalty(-1, 0);
		p.normalize = Normalize.signature;
		p.similarity = Similarity.strings;
		
		Report report = GeneralAlignment.align(p);
		if (Double.compare(report.score, Double.NaN) == 0) { 
			System.out.println("WTF?: " + _signature.signature().size() + " -- " + instance.sequence().size());
		}
		return report.score;
	}
	
	/**
	 * Return an overlap score between this cluster and the
	 * given cluster.
	 * @param c
	 * @return
	 */
	public double sim(Cluster c) { 
		// first find the intersection of these two clusters...
		// use the unique id for quick testing.
		Set<Integer> id1 = new TreeSet<Integer>();
		Set<Integer> id2 = new TreeSet<Integer>();
		
		for (Instance i : _instances) 
			id1.add(i.uniqueId());
		
		for (Instance i : c._instances) 
			id2.add(i.uniqueId());

		id1.retainAll(id2);
		return ((double) 2.0 * id1.size()) / ((double) (_instances.size() + c._instances.size()));
	}
}
