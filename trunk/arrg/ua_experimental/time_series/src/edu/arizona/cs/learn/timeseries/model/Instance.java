package edu.arizona.cs.learn.timeseries.model;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.List;

import org.apache.commons.math.stat.clustering.Clusterable;
import org.apache.log4j.Logger;

import edu.arizona.cs.learn.timeseries.evaluation.cluster.Clustering;
import edu.arizona.cs.learn.timeseries.model.symbols.Symbol;

public class Instance implements Clusterable<Instance> {
	private static Logger logger = Logger.getLogger(Instance.class);
	private int _id;
	private int _uniqueId;
	private String _name;
	
	private List<Interval> _intervals;
	private List<Symbol> _sequence;

	public Instance(String name, int id, List<Symbol> seq) {
		this(name, id, null, seq);
	}

	public Instance(String name, int id, List<Interval> intervals, List<Symbol> seq) {
		_name = name;
		_id = id;
		
		_intervals = intervals;
		_sequence = seq;
	}

	public String name() {
		return _name;
	}

	public int id() {
		return _id;
	}

	public int uniqueId() {
		return _uniqueId;
	}

	public void uniqueId(int uniqueId) {
		_uniqueId = uniqueId;
	}

	public void shuffle() {
		Collections.shuffle(this._sequence);
	}

	public List<Interval> intervals() { 
		return _intervals;
	}
	
	public List<Symbol> sequence() {
		return this._sequence;
	}

	public void reverse() {
		Collections.reverse(this._sequence);
	}

	public Instance copy() {
		List<Symbol> seq = new ArrayList<Symbol>();
		for (Symbol obj : _sequence) {
			seq.add(obj.copy());
		}

		Instance copy = new Instance(_name, _id, _intervals, seq);
		copy.uniqueId(_uniqueId);
		return copy;
	}

	public Instance centroidOf(Collection<Instance> cluster) {
		if (cluster == null) {
			logger.error("NULL cluster");
			return null;
		}

		if (cluster.size() == 0) {
			logger.error("Empty cluster");
			return null;
		}

		List<Instance> instances = new ArrayList<Instance>(cluster);
		double[] sumDistance = new double[instances.size()];

		for (int i = 0; i < instances.size(); i++) {
			Instance i1 = (Instance) instances.get(i);
			for (int j = i + 1; j < instances.size(); j++) {
				Instance i2 = (Instance) instances.get(j);

				double d = Clustering.distances[i1.uniqueId()][i2.uniqueId()];
				sumDistance[i] += d;
				sumDistance[j] += d;
			}
		}

		int index = 0;
		double minDistance = (1.0D / 0.0D);
		for (int i = 0; i < instances.size(); i++) {
			if (sumDistance[i] < minDistance) {
				index = i;
				minDistance = sumDistance[i];
			}
		}
		return (Instance) instances.get(index);
	}

	public double distanceFrom(Instance i) {
		return Clustering.distances[this._uniqueId][i.uniqueId()];
	}
}