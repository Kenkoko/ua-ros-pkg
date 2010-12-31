package edu.arizona.cs.learn.timeseries.clustering.kmeans;

import java.util.ArrayList;
import java.util.List;

import edu.arizona.cs.learn.algorithm.alignment.GeneralAlignment;
import edu.arizona.cs.learn.algorithm.alignment.Normalize;
import edu.arizona.cs.learn.algorithm.alignment.Params;
import edu.arizona.cs.learn.algorithm.alignment.Report;
import edu.arizona.cs.learn.algorithm.alignment.Similarity;
import edu.arizona.cs.learn.timeseries.model.Instance;
import edu.arizona.cs.learn.timeseries.model.Signature;

public class Cluster {

	private int _id;
	
	private List<Integer> _indexes;
	private List<Instance> _instances;
	
	private Signature _signature;
	
	public Cluster(int id) { 
		_id = id;
		clear();
	}
	
	public int id() { 
		return _id;
	}
	
	public List<Integer> indexes() { 
		return _indexes;
	}
	
	public void clear() { 
		_signature = new Signature(_id + "");
		_indexes = new ArrayList<Integer>();
		_instances = new ArrayList<Instance>();
	}
	
	public void add(int index, Instance instance) {
		_indexes.add(index);
		_instances.add(instance);
	}

	public void finish() { 
		if (_indexes.size() == 0) {
			System.out.println("Shit --- a cluster is empty");
			return;
		}
		
		for (int i = 1; i < _instances.size(); ++i) { 
			if (i % 10 == 0) 
				_signature = _signature.prune(3);
			_signature.update(_instances.get(i-1).sequence());
		}

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
		if (_indexes.size() == 0) {
			return Math.random();
		}
		
		Params p = new Params(_signature.signature(), instance.sequence());
		p.setMin(0,0);
		p.setBonus(1, 0);
		p.setPenalty(-1, 0);
		p.normalize = Normalize.signature;
		p.similarity = Similarity.strings;
		
		Report report = GeneralAlignment.align(p);
		return report.score;
	}
}
