package edu.arizona.cs.learn.algorithm.alignment.model;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.List;

import org.apache.commons.math.stat.clustering.Clusterable;
import org.apache.log4j.Logger;

import edu.arizona.cs.learn.timeseries.evaluation.cluster.Clustering;
import edu.arizona.cs.learn.timeseries.model.Interval;
import edu.arizona.cs.learn.util.Utils;

public class Instance implements Clusterable<Instance> {
	private static Logger logger = Logger.getLogger(Instance.class);
	private int _id;
	private int _uniqueId;
	private String _name;
	private List<WeightedObject> _sequence;

	public Instance(String name, int id, List<WeightedObject> seq) {
		this._name = name;
		this._id = id;
		this._sequence = seq;
	}

	public String name() {
		return this._name;
	}

	public int id() {
		return this._id;
	}

	public void uniqueId(int uniqueId) {
		this._uniqueId = uniqueId;
	}

	public int uniqueId() {
		return this._uniqueId;
	}

	public void shuffle() {
		Collections.shuffle(this._sequence);
	}

	public List<WeightedObject> sequence() {
		return this._sequence;
	}

	public void reverse() {
		Collections.reverse(this._sequence);
	}

	public Instance copy() {
		List<WeightedObject> seq = new ArrayList<WeightedObject>();
		for (WeightedObject obj : this._sequence) {
			boolean add = true;
			for (Interval interval : obj.key().getIntervals()) {
				for (String exclude : Utils.testExcludeSet) {
					if ((!interval.name.endsWith(exclude))
							&& (!interval.name.startsWith(exclude)))
						continue;
					add = false;
					break;
				}

				if (!add) {
					break;
				}
			}
			if (add) {
				seq.add(obj);
			}

		}

		Instance copy = new Instance(this._name, this._id, seq);
		copy.uniqueId(this._uniqueId);
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