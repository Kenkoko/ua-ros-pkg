package edu.arizona.cs.learn.timeseries.clustering.kmeans;

import java.io.File;
import java.io.PrintStream;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.TreeMap;
import java.util.concurrent.Callable;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;

import edu.arizona.cs.learn.timeseries.clustering.ClusteringResults;
import edu.arizona.cs.learn.timeseries.experiment.SyntheticExperiments;
import edu.arizona.cs.learn.timeseries.model.Instance;
import edu.arizona.cs.learn.timeseries.model.SequenceType;
import edu.arizona.cs.learn.util.RandomFile;
import edu.arizona.cs.learn.util.Utils;

public class KMeans {

	private ExecutorService _execute;

	private int _k;
	private int _maxIter;
	
	private ClusterInit _init;
	private ClusterType _type;
	
	public KMeans(int k, int maxIter, ClusterInit init, ClusterType type) { 
		_k = k;
		_maxIter = maxIter;
		
		_init = init;
		_type = type;
	}
	
	/**
	 * Use the KMeans to perform the clustering and the KMeans++ algorithm
	 * to select initial clusters.
	 * @param instances
	 */
	public ClusteringResults cluster(final List<Instance> instances) { 
		_execute = Executors.newFixedThreadPool(Utils.numThreads);
		
		// Make ground truth clusters....
		Map<String,Cluster> gtClusters = new HashMap<String,Cluster>();
		for (Instance instance : instances) { 
			Cluster c = gtClusters.get(instance.name());
			if (c == null) { 
				c = _type.make(instance.name(), gtClusters.size());
				gtClusters.put(instance.name(), c);
			}
			c.add(instance);
		}
		List<Cluster> groundTruth = new ArrayList<Cluster>(gtClusters.values());
		
		
		// assign uniqueIds to each of the instances....
		for (int i = 0; i < instances.size(); ++i) { 
			instances.get(i).uniqueId(i+100);
		}
		
		final List<Cluster> clusters = new ArrayList<Cluster>();
		for (int i = 0; i < _k; ++i) 
			clusters.add(_type.make(i));
		
		// Initialize the first clusters according to the method given.
		_init.pickCenters(clusters, instances);
		
		int iteration = 1;
		boolean changing = true;
		while (changing && iteration <= _maxIter) {
			changing = false;
			
			System.out.println("Iteration: " + iteration);

			finishClusters(clusters);
			
			// Fill the map containing the current mapping of instances to clusters
			final Map<Integer,Integer> clusterMap = new TreeMap<Integer,Integer>();
			for (Cluster c : clusters) { 
				for (Instance instance : c.instances())
					clusterMap.put(instance.uniqueId(), c.id());
			}

			// Initialize the new clusters.
			List<Cluster> tmp = new ArrayList<Cluster>();
			for (int i = 0; i < _k; ++i) 
				tmp.add(_type.make(i));
			
			List<Future<ClusterDistance>> futureList = new ArrayList<Future<ClusterDistance>>();
			for (int i = 0; i < instances.size(); ++i) 
				futureList.add(_execute.submit(new ClusterDistance(instances.get(i), clusters)));
			
			for (Future<ClusterDistance> future : futureList) { 
				try { 
					ClusterDistance dc = future.get();
					tmp.get(dc.clusterId).add(dc.instance);
					
					Integer newId = clusterMap.get(dc.instance.uniqueId());
					if (newId == null) {
						changing = true;
						System.out.println("\t\tNew Assignment " + dc.instance.uniqueId());
					} else if (dc.clusterId != newId) {
						changing = true;
						System.out.println("\t\tChanged cluster: " + dc.instance.uniqueId() + " -- " + dc.clusterId + " -- " + newId);
					}
				} catch (Exception e) { 
					e.printStackTrace();
				}
			}

			clusters.clear();
			clusters.addAll(tmp);
			
			++iteration;
		}
		_execute.shutdown();
		
		// now we can measure performance
		ClusteringResults results = new ClusteringResults(groundTruth, clusters);
		results.accordance(instances);
		return results;
	}
	
	public void printClusters(PrintStream out, List<Cluster> clusters) {
		out.println("Clusters...");
		for (Cluster c : clusters) { 
			out.println("  Cluster: " + c.id());
			out.print("  \t[");
			for (Instance instance : c.instances()) { 
				out.print(instance.name()+ "-" + instance.id() + ",");
			}
			out.println("]");
		}
	}
	
	/**
	 * Iterate over all of the clusters and call finish on them so 
	 * that we can build the correct signature.
	 * @param clusters
	 */
	public void finishClusters(List<Cluster> clusters) { 
		// Call finish on all of the clusters....
		List<Future<ClusterFinish>> finishList = new ArrayList<Future<ClusterFinish>>();
		for (Cluster c : clusters) 
			finishList.add(_execute.submit(new ClusterFinish(c)));
		
		for (Future<ClusterFinish> future : finishList) {
			try { 
				ClusterFinish finish = future.get();
			} catch (Exception e) { 
				e.printStackTrace();
			}
		}
	}
	
	public static void main(String[] args) throws Exception { 
		// perform a test with a small subset of the total problem.
		if (args.length < 3) {
			System.out.println("Usage: <prefix/synthetic> <ClusterType> <ClusterInit> ...");
			System.out.println("  ClusterTypes: signature, medoid");
			System.out.println("  ClusterInits: random, kPlusPlus, supervised1, supervised5, supervised10");
			return;
		}
		String dataType = args[0];
		
		List<ClusterType> types = new ArrayList<ClusterType>();
		String[] typeTokens = args[1].split("[,]");
		for (String tok : typeTokens)
			types.add(ClusterType.valueOf(tok));
		
		List<ClusterInit> inits = new ArrayList<ClusterInit>();
		String[] initTokens = args[2].split("[,]");
		for (String tok : initTokens)
			inits.add(ClusterInit.valueOf(tok));

		Utils.EXPERIMENTS = Integer.parseInt(args[3]);
		
		if ("synthetic".equals(dataType)) { 
			if (args.length != 8) {
				System.out.println("Synthetic usage: synthetic <ClusterType> <ClusterInit> #Experiments " +
						"#Repeats <Means> <Percents> <Lengths>");
				System.out.println("  ClusterTypes: signature, medoid");
				System.out.println("  ClusterInits: random, kPlusPlus, supervised1, supervised5, supervised10");
				return;
			}
			
			Utils.LIMIT_RELATIONS = true;
			Utils.WINDOW = 2;
			
			int repeats = Integer.parseInt(args[4]);
			
			List<Double> means = new ArrayList<Double>();
			String[] meanTokens = args[5].split("[,]");
			for (String tok : meanTokens)
				means.add(Double.parseDouble(tok));
			
			List<Double> pcts = new ArrayList<Double>();
			String[] pctTokens = args[6].split("[,]");
			for (String tok : pctTokens)
				pcts.add(Double.parseDouble(tok));
			
			List<Integer> lengths = new ArrayList<Integer>();
			String[] lengthTokens = args[7].split("[,]");
			for (String tok : lengthTokens)
				lengths.add(Integer.parseInt(tok));
			
			cluster(types, inits, repeats, lengths, means, pcts);
		} else { 
			if (args.length != 5) {
				System.out.println("Other usage: dataType <ClusterType> <ClusterInit> #Experiments Directory");
				System.out.println("  ClusterTypes: signature, medoid");
				System.out.println("  ClusterInits: random, kPlusPlus, supervised1, supervised5, supervised10");
				return;
			}
			
			// We are dealing with deterministic data
			// and therefore cannot generate random permutations
			// of it.
			String directory = args[4];
			for (ClusterInit init : inits) { 
				for (ClusterType type : types) { 
					PrintStream out = new PrintStream(new File("logs/cluster-synthetic-" + 
							type + "-" + init + "-" + dataType + ".csv"));
					out.println("cluster_type,cluster_init,dataType,run,tp,fp,fn,tn,accuracy");
					doCluster(out, directory, dataType, init, type, dataType);
				}
			}
		}
		
//		List<Instance> all = new ArrayList<Instance>();
//		List<Instance> set1 = Utils.sequences("A", "data/input/ww3d-jump-over.lisp", SequenceType.allen);
//		List<Instance> set2 = Utils.sequences("B", "data/input/ww3d-jump-on.lisp", SequenceType.allen);
//		List<Instance> set3 = Utils.sequences("C", "data/input/ww3d-left.lisp", SequenceType.allen);
//		List<Instance> set4 = Utils.sequences("D", "data/input/ww3d-right.lisp", SequenceType.allen);
//		List<Instance> set5 = Utils.sequences("E", "data/input/ww3d-push.lisp", SequenceType.allen);
////		List<Instance> set6 = Utils.sequences("F", "data/input/ww3d-approach.lisp", SequenceType.allen);
//
////		for (int i = 0; i < 10; ++i) { 
////			all.add(set1.get(i));
////			all.add(set2.get(i));
////			all.add(set3.get(i));
////			all.add(set4.get(i));
////			all.add(set5.get(i));
//////			all.add(set6.get(i));
////		}
//		
//		all.addAll(set1);
//		all.addAll(set2);
//		all.addAll(set3);
//		all.addAll(set4);
//		all.addAll(set5);
////		all.addAll(set6);
//
//		PrintStream out = new PrintStream(new File("logs/synthetic-clustering-ww3d.csv"));
//		for (ClusterInit init : ClusterInit.values()) { 
//			for (ClusterType type : ClusterType.values()) { 
//				KMeans kmeans = new KMeans(5, 20, init, type);
//				kmeans.cluster(all);
//			}
//		}
	}


	/**
	 * 
	 * @param types
	 * @param inits
	 * @param repeats
	 * @param episodeLengths
	 * @param means
	 * @param pcts
	 * @throws Exception
	 */
	public static void cluster(List<ClusterType> types, List<ClusterInit> inits, int repeats,
			List<Integer> episodeLengths, List<Double> means, List<Double> pcts) throws Exception { 
		String pid = RandomFile.getPID();

		for (double pct : pcts) { 
			for (double mean : means) { 
				for (int length : episodeLengths) { 
					for (int i = 0; i < repeats; ++i) { 
						SyntheticExperiments.generateClass(pid, "f", 0, 0, length);
						SyntheticExperiments.generateClass(pid, "g", mean, pct, length);

						String params = pct + "-" + mean + "-" + length;
						String rowString = pct + "," + mean + "," + length;
						for (ClusterInit init : inits) { 
							for (ClusterType type : types) { 
								PrintStream out = new PrintStream(new File("logs/cluster-synthetic-" + 
										type + "-" + init + "-" + params + ".csv"));
								out.println("cluster_type,cluster_init,pct,mean,length,run,tp,fp,fn,tn,accuracy");
								doCluster(out, "/tmp/niall-" + pid + "/", "niall", init, type, rowString);
							}
						}
					}
				}
			}
		}
	}	
	
	/**
	 * Run some number of clusters through and output the results.
	 * @param out
	 * @param dir
	 * @param prefix
	 * @param init
	 * @param type
	 * @param rowInfo
	 */
	public static void doCluster(PrintStream out, String dir, String prefix, 
			ClusterInit init, ClusterType type, String rowInfo) { 
		Map<String,List<Instance>> data = Utils.load(dir, prefix, SequenceType.allen);
		List<Instance> all = new ArrayList<Instance>();
		for (String name : data.keySet()) { 
			all.addAll(data.get(name));
		}

		// now we repeat the experiment x number of times....
		for (int i = 0; i < Utils.EXPERIMENTS; ++i) { 
			KMeans kmeans = new KMeans(data.keySet().size(), 20, init, type);
			ClusteringResults result = kmeans.cluster(all);
			out.println(type + "," + init + "," + rowInfo + "," + i + 
					"," + result.tp + "," + result.fp + "," + result.fn + "," + result.tn + 
					"," + result.accuracy);
		}
	}
	
}

/**
 * Iterate over all of the clusters and find the one with 
 * the smallest distance to us.
 * @author wkerr
 *
 */
class ClusterDistance implements Callable<ClusterDistance> {
	private List<Cluster> _clusters;
	
	public Instance instance;
	public int clusterId;

	public boolean changing = false;
	
	public ClusterDistance(Instance instance, List<Cluster> clusters) {
		this.instance = instance;

		_clusters = clusters;
	}

	@Override
	public ClusterDistance call() throws Exception {
		double minD = Double.POSITIVE_INFINITY;
		clusterId = -1;
		for (Cluster c : _clusters) { 
			double d = c.distance(instance);
			if (d < minD) { 
				minD = d;
				clusterId = c.id();
			}
		}
		if (clusterId == -1)
			throw new RuntimeException("Impossible: ---" + _clusters.size());
		return this;
	} 
}

class ClusterFinish implements Callable<ClusterFinish> {
	private Cluster _cluster;
	
	public ClusterFinish(Cluster c) { 
		_cluster = c;
	}

	@Override
	public ClusterFinish call() throws Exception {
		_cluster.computeCentroid();
		return this;
	}
	
}

