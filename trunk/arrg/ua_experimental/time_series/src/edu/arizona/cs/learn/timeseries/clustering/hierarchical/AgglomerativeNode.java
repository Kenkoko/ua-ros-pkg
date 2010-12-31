package edu.arizona.cs.learn.timeseries.clustering.hierarchical;

import java.util.ArrayList;
import java.util.List;

import edu.arizona.cs.learn.algorithm.alignment.GeneralAlignment;
import edu.arizona.cs.learn.algorithm.alignment.Normalize;
import edu.arizona.cs.learn.algorithm.alignment.Params;
import edu.arizona.cs.learn.algorithm.alignment.Report;
import edu.arizona.cs.learn.algorithm.alignment.Similarity;
import edu.arizona.cs.learn.timeseries.model.Instance;
import edu.arizona.cs.learn.timeseries.model.Signature;

public class AgglomerativeNode {

	public static boolean BUILD_SIGNATURE = false;
	
	private int _index;
	private Instance _instance;

	private AgglomerativeNode _left;
	private AgglomerativeNode _right;
	
	private double _distance;
	private Signature _signature;
	private List<Instance> _instances;
	private List<Integer> _indexes;
	
	private boolean _isLeaf;
	
	/**
	 * Construct a new node from a single instance...i.e. a Leaf
	 * @param index
	 * @param instance
	 */
	public AgglomerativeNode(int index, Instance instance) { 
		_index = index;
		_instance = instance;
		_distance = 0.0;
		_isLeaf = true;
		
		_indexes = new ArrayList<Integer>();
		_indexes.add(_index);
		
		_instances = new ArrayList<Instance>();
		_instances.add(instance);

		if (BUILD_SIGNATURE) {
			_signature = new Signature("node");
			_signature.update(_instance.sequence());
		}
	}

	/**
	 * Construct a new Node from two existing nodes...i.e. a Cluster
	 * @param left
	 * @param right
	 */
	public AgglomerativeNode(AgglomerativeNode left, AgglomerativeNode right, double distance) { 
		_left = left;
		_right = right;
		_distance = _left.distance() + _right.distance() + distance;
		_isLeaf = false;
		
		_indexes = new ArrayList<Integer>();
		_indexes.addAll(_left.indexes());
		_indexes.addAll(_right.indexes());
		
		_instances = new ArrayList<Instance>();
		_instances.addAll(left._instances);
		_instances.addAll(right._instances);
		
		if (BUILD_SIGNATURE) {
			_signature = new Signature("node");
			for (Instance i : _instances) { 
				_signature.update(i.sequence());
			}
		}
	}
	
	
	/**
	 * Return the indexes that make up this node.
	 * @return
	 */
	public List<Integer> indexes() {
		return _indexes;
	}
	
	/**
	 * Return the label of this Node.  Only valid for
	 * Leafs
	 * @return
	 */
	public String label() { 
		if (!_isLeaf)
			throw new RuntimeException("label() only valid for leaf nodes");
		return _instance.name() + "-" + _instance.id();
	}
	
	/**
	 * Return the left node
	 * Only valid for Clusters
	 * @return
	 */
	public AgglomerativeNode left() { 
		if (_isLeaf)
			throw new RuntimeException("left() only valid for cluster nodes");
		return _left;
	}
	
	/**
	 * Return the right node
	 * Only valid for Clusters
	 * @return
	 */
	public AgglomerativeNode right() { 
		if (_isLeaf)
			throw new RuntimeException("right() only valid for cluster nodes");
		return _right;
	}
	
	/**
	 * Return the distance between the left node and the right node.
	 * If this is a leaf, then the distance is 0.
	 * @return
	 */
	public double distance() { 
		return _distance;
	}
	
	/**
	 * Return whether or not this node is a leaf;
	 * @return
	 */
	public boolean isLeaf() { 
		return _isLeaf;
	}
	
	/**
	 * Call the specific method and return the distance between nodes
	 * @param n2
	 * @param matrix
	 * @param method
	 * @return
	 */
	public double distance(AgglomerativeNode n2, double[][] matrix, String method) { 
		if ("single".equals(method))
			return singleLinkage(n2, matrix);
		if ("complete".equals(method))
			return completeLinkage(n2, matrix);
		if ("average".equals(method))
			return averageLinkage(n2, matrix);
		if ("signature".equals(method))
			return signature(n2, matrix);

		throw new RuntimeException("Unknown method: " + method);
	}

	/**
	 * Return the distance between the closest sequences 
	 * in the two nodes.
	 * @param n
	 * @param matrix
	 * @return
	 */
	public double singleLinkage(AgglomerativeNode n2, double[][] matrix) { 
		double min = Double.POSITIVE_INFINITY;
		for (Integer i : indexes()) { 
			for (Integer j : n2.indexes()) { 
//				logger.debug("Distance: " + i + " " + j + " -- " + matrix[i][j]);
				min = Math.min(min, matrix[i][j]);
			}
		}
//		logger.debug("    MIN: " + min);
		return min;
	}

	/**
	 * Return the distance between the two farthest sequences
	 * in the two nodes
	 * @param n2
	 * @param matrix
	 * @return
	 */
	public double completeLinkage(AgglomerativeNode n2, double[][] matrix) { 
		double max = Double.NEGATIVE_INFINITY;
		for (Integer i : indexes()) { 
			for (Integer j : n2.indexes()) { 
				max = Math.max(max, matrix[i][j]);
			}
		}
		return max;
	}

	/**
	 * Return the average distance between the cross product of
	 * all the sequences in each node
	 * @param n2
	 * @param matrix
	 * @return
	 */
	public double averageLinkage(AgglomerativeNode n2, double[][] matrix) { 
		double sum = 0;
		double count = 0;
		for (Integer i : indexes()) { 
			for (Integer j : n2.indexes()) { 
				sum += matrix[i][j];
				count += 1;
			}
		}
		return sum / count;
	}
	
	/**
	 * @param n2
	 * @param matrix
	 * @return
	 */
	public double signature(AgglomerativeNode n2, double[][] matrix) { 
		if (isLeaf() && n2.isLeaf())
			return singleLinkage(n2, matrix);
		
		Params p = new Params(_signature.signature(), n2._signature.signature());
		p.setMin(0, 0);
		p.setBonus(1, 1);
		p.setPenalty(0, 0);
		p.normalize = Normalize.signature;
		p.similarity = Similarity.strings;
		
		Report r = GeneralAlignment.align(p);
		return r.score;
	}
}
