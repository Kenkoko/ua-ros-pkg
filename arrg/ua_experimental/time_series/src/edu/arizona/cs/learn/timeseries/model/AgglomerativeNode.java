package edu.arizona.cs.learn.timeseries.model;

import java.util.ArrayList;
import java.util.List;

import org.apache.log4j.Logger;

import edu.arizona.cs.learn.algorithm.alignment.model.Instance;

public class AgglomerativeNode {
	private static Logger logger = Logger.getLogger(AgglomerativeNode.class);

	public List<Integer> sequenceIndexes;
	public Signature signature;

	public AgglomerativeNode(int index, Instance instance) { 
		signature = new Signature(instance.name());
		signature.update(instance.sequence());

		sequenceIndexes = new ArrayList<Integer>();
		sequenceIndexes.add(index);
	}

	/**
	 * This constructor is called when you want to combine
	 * two nodes into a single node
	 * @param node1
	 * @param node2
	 */
	public AgglomerativeNode(AgglomerativeNode node1, AgglomerativeNode node2) { 
		logger.debug("Combining: ");
		logger.debug("  " + node1.sequenceIndexes);
		logger.debug("  " + node2.sequenceIndexes);
		
		signature = node1.signature;
		signature.merge(node2.signature);

		sequenceIndexes = new ArrayList<Integer>();
		sequenceIndexes.addAll(node1.sequenceIndexes);
		sequenceIndexes.addAll(node2.sequenceIndexes);
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
		for (Integer i : sequenceIndexes) { 
			for (Integer j : n2.sequenceIndexes) { 
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
		for (Integer i : sequenceIndexes) { 
			for (Integer j : n2.sequenceIndexes) { 
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
		for (Integer i : sequenceIndexes) { 
			for (Integer j : n2.sequenceIndexes) { 
				sum += matrix[i][j];
				count += 1;
			}
		}
		return sum / count;
	}
}
