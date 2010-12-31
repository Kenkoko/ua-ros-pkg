package edu.arizona.cs.learn.timeseries.evaluation.cluster;

import java.io.File;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Random;

import org.apache.commons.collections15.Bag;
import org.apache.commons.collections15.bag.HashBag;
import org.apache.commons.math.stat.clustering.Cluster;
import org.apache.commons.math.stat.clustering.KMeansPlusPlusClusterer;
import org.apache.log4j.Logger;

import edu.arizona.cs.learn.timeseries.distance.Distances;
import edu.arizona.cs.learn.timeseries.model.Instance;
import edu.arizona.cs.learn.timeseries.model.SequenceType;
import edu.arizona.cs.learn.util.Utils;

public class Clustering {
    private static Logger logger = Logger.getLogger(Clustering.class);

	/** Before using this variable, make sure that you call buildDistanceMatrix */
	public static double[][] distances = null;
	
	public Clustering() { 
		
	}
	
	/**
	 * Perform a KMeans++ clustering of the instances given by the prefix
	 * and the generator.
	 * @param prefix
	 * @param generator
	 */
	public void cluster(String prefix, SequenceType type) { 
		List<Instance> all = new ArrayList<Instance>();
		int classSize = 0;
		
    	for (File f : new File("data/input/").listFiles()) {
    		if (f.getName().startsWith(prefix)) { 
    			String name = f.getName();
    			String className = name.substring(0, name.indexOf(".lisp"));

    			List<Instance> list = Utils.sequences(className, f.getAbsolutePath(), type);
    			System.out.print(className + " range " + all.size() + " to ");
    			all.addAll(list);
    			System.out.println(all.size());
    			classSize++;
    		}
    	}
    	
    	Distances.distances(all);
		
		boolean broken = true;
		while (broken) { 
			try { 
				broken = false;
				logger.debug("Number of episodes: " + all.size());
				Random r = new Random();
				KMeansPlusPlusClusterer<Instance> clusterer = new KMeansPlusPlusClusterer<Instance>(r);
				List<Cluster<Instance>> clusters = clusterer.cluster(all, classSize, 50);
				printClusters(clusters);
			} catch (Exception e) {
				e.printStackTrace();
				broken = true;
			}
		}
	}	
	
	/**
	 * Perform a KMeans++ clustering of the instances given by the prefix
	 * and the generator.
	 * @param prefix
	 * @param generator
	 */
	public void cluster(String prefix, String matrixFile, SequenceType sequenceType) { 
		List<Instance> all = new ArrayList<Instance>();
		Map<String,List<Instance>> map = Utils.load(prefix, sequenceType);

		int uniqueId = 0;
		for (String key : map.keySet()) { 
			System.out.print(key + " range " + all.size() + " to ");
			List<Instance> list = map.get(key);
			for (Instance instance : list) { 
				instance.uniqueId(uniqueId);
				all.add(instance);
				
				++uniqueId;
			}
			System.out.println(all.size());
		}
		int classSize = map.keySet().size();
    	distances = Distances.load(matrixFile, all);
    	
		boolean broken = true;
		while (broken) { 
			try { 
				broken = false;
				logger.debug("Number of episodes: " + all.size());
				Random r = new Random();
				
				logger.debug("Distances: " + distances);
				KMeansPlusPlusClusterer<Instance> clusterer = new KMeansPlusPlusClusterer<Instance>(r);
				List<Cluster<Instance>> clusters = clusterer.cluster(all, classSize, 50);
				printClusters(clusters);
			} catch (Exception e) {
				e.printStackTrace();
				broken = true;
			}
		}
	}		
	/**
	 * Print out the breakdown of the different clusters.
	 * @param clusters
	 */
	public static void printClusters(List<Cluster<Instance>> clusters) { 
		int overallRight = 0;
		int overallSum = 0;
		
		for (Cluster<Instance> cluster : clusters) { 
			logger.debug("******** Cluster ********");
			Bag<String> bag = new HashBag<String>();
			for (Instance c : cluster.getPoints()) { 
				bag.add(c.name());
			}

			int localMax = 0;
			for (String s : bag.uniqueSet()) { 
				int count = bag.getCount(s);
				logger.debug("\t" + s + " - " + count + " " + ((double) count / bag.size()));
				
				localMax = Math.max(localMax, count);
				overallSum += count;
			}
			overallRight += localMax;
		}
		
		logger.debug("Clustering Score: " + ((double) overallRight / (double) overallSum));
	}

	/**
	 * Iterate through and generate all of the distance matrices that are 
	 * requested.
	 * @param prefixes
	 * @param sList
	 */
	public static void init(List<String> prefixes, List<SequenceType> sList) { 
		Clustering c = new Clustering();
		for (String prefix : prefixes) { 
			for (SequenceType type : sList) { 
				double[][] matrix = Distances.distances(prefix, type);
				Distances.save("distances/" + prefix + "-" + type + ".csv", matrix);
			}
		}
	}
	
	public static void cluster(List<String> prefixes, List<SequenceType> sList) { 
		for (String prefix : prefixes) { 
			for (SequenceType type : sList) { 
				// generate 5 random clusterings just to make sure....
				Clustering c  = new Clustering();
				c.cluster(prefix, type);
				
				double[][] matrix = Distances.distances(prefix, type);
				Distances.save("distances/" + prefix + "-" + type + ".csv", matrix);
			}
		}
	}
	
	public static void main(String[] args) { 
		String prefix = "ww3d";
		List<Instance> all = new ArrayList<Instance>();
    	for (File f : new File("data/input/").listFiles()) {
    		if (f.getName().startsWith(prefix)) { 
    			String name = f.getName();
    			String className = name.substring(0, name.indexOf(".lisp"));

    			List<Instance> list = Utils.sequences(className, f.getAbsolutePath(), SequenceType.starts);
    			System.out.print(className + " range " + all.size() + " to ");
    			all.addAll(list);
    			System.out.println(all.size());
    		}
    	}
//    	System.out.println("(n x n) = (" + all.size() + " " + all.size() + ")");
//    	System.out.println("   " + all.size()*all.size());
//		
		Clustering c = new Clustering();
//		double[][] matrix = c.buildDistanceMatrix(all);
//		c.writeMatrix("/tmp/ + " + prefix + "-distance-matrix.csv", matrix);
		
//		c.writeDistanceMatrix("ww2d", SequenceFactory.propGenerator("starts", Interval.starts));
//		c.writeDistanceMatrix("ww2d", SequenceFactory.sortedAllenGenerator());
//		c.cluster("ww2d", SequenceFactory.propGenerator("starts", Interval.starts));
//		c.cluster("ww2d", SequenceFactory.sortedAllenGenerator());
//		c.cluster("ww3d", gen);

//		c.cluster("ww2d", "distances/ww2d-starts.csv", SequenceFactory.propGenerator("starts", Interval.starts));
//		c.cluster("ww2d", "distances/ww2d-ends.csv", SequenceFactory.propGenerator("ends", Interval.ends));
//		c.cluster("ww2d", "distances/ww2d-startsEnds.csv", SequenceFactory.startEndGenerator());
//		c.cluster("ww2d", "distances/ww2d-allen.csv", SequenceFactory.sortedAllenGenerator());

		String dataPrefix = "/Users/wkerr/Dropbox/papers/dissertation/uathesis/data/distances/";
		c.cluster("ww3d", dataPrefix + "ww3d-starts.csv", SequenceType.starts);
		c.cluster("ww3d", dataPrefix + "ww3d-ends.csv", SequenceType.ends);
		c.cluster("ww3d", dataPrefix + "ww3d-startsEnds.csv", SequenceType.startsEnds);
		c.cluster("ww3d", dataPrefix + "ww3d-allen.csv", SequenceType.allen);
		
		
//		double[][] a = c.buildDistanceMatrix("ww3d", SequenceFactory.propGenerator("starts", Interval.starts));
//		double[][] b = c.loadDistanceMatrix("distances/ww3d-starts.csv", all);
//		
//		for (int i = 0; i < a.length; ++i) { 
//			for (int j = 0; j < a[i].length; ++j) { 
//				if (Double.compare(a[i][j], b[i][j]) != 0) { 
//					throw new RuntimeException("Unequal: " + i + " " + j);
//				}
//			}
//		}
	}
}
