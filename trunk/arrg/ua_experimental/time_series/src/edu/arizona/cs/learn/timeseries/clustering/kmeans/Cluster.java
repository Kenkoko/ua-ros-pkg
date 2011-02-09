package edu.arizona.cs.learn.timeseries.clustering.kmeans;

import java.util.ArrayList;
import java.util.List;
import java.util.Set;
import java.util.TreeSet;

import edu.arizona.cs.learn.algorithm.alignment.Normalize;
import edu.arizona.cs.learn.timeseries.model.Instance;

public abstract class Cluster {
	/** Allow the user to specify whether or not the cluster
	 * should normalize the signature distance.
	 */
	public static Normalize normalize = Normalize.signature;
	

	private int _id;
	private String _name;
	
	protected List<Instance> _instances;

	/**
	 * Initialize a new cluster using the id of the cluster
	 * as the name.
	 * @param id
	 */
	public Cluster(int id) { 
		this(id+"", id);
	}
	
	/**
	 * Initialize a new cluster giving it a name and an id.
	 * @param name
	 * @param id
	 */
	public Cluster(String name, int id) { 
		_id = id;
		_name = name;
		clear();
	}
	
	/**
	 * return the id of the cluster
	 * @return
	 */
	public int id() { 
		return _id;
	}
	
	/**
	 * return the name of the cluster.
	 * @return
	 */
	public String name() { 
		return _name;
	}
	
	/**
	 * Clear this cluster in preparation for new instances.
	 */
	public void clear() { 
		_instances = new ArrayList<Instance>();
	}
	
	/**
	 * Add an instance to this cluster.
	 * @param instance
	 */
	public void add(Instance instance) {
		_instances.add(instance);
	}

	/**
	 * Return all of the instances that are part of this cluster.
	 * @return
	 */
	public List<Instance> instances() { 
		return _instances;
	}
	
	/**
	 * This method is called after all of the instances are assigned to a 
	 * cluster.  It provides an opportunity for the cluster to 
	 * construct the centroid.
	 */
	public abstract void computeCentroid();
	
	/**
	 * Return the distance between this signature for this
	 * cluster and the given instance.
	 * @param instance
	 * @return
	 */
	public abstract double distance(Instance instance);
	
	/**
	 * Does this cluster contain the given instance.
	 * Returning true if the instance's reference is in
	 * the list of instances
	 * @param instance
	 * @return
	 */
	public boolean contains(Instance instance) { 
		return _instances.contains(instance);
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
