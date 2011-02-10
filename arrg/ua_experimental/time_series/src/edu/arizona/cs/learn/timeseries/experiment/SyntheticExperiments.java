package edu.arizona.cs.learn.timeseries.experiment;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.PrintStream;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Map;
import java.util.Random;

import org.apache.commons.math.stat.descriptive.SummaryStatistics;

import edu.arizona.cs.learn.timeseries.classification.Classifier;
import edu.arizona.cs.learn.timeseries.classification.Classify;
import edu.arizona.cs.learn.timeseries.classification.ClassifyParams;
import edu.arizona.cs.learn.timeseries.clustering.ClusteringResults;
import edu.arizona.cs.learn.timeseries.clustering.kmeans.ClusterInit;
import edu.arizona.cs.learn.timeseries.clustering.kmeans.ClusterType;
import edu.arizona.cs.learn.timeseries.clustering.kmeans.KMeans;
import edu.arizona.cs.learn.timeseries.evaluation.BatchStatistics;
import edu.arizona.cs.learn.timeseries.evaluation.SplitAndTest;
import edu.arizona.cs.learn.timeseries.model.Instance;
import edu.arizona.cs.learn.timeseries.model.SequenceType;
import edu.arizona.cs.learn.timeseries.prep.SymbolicData;
import edu.arizona.cs.learn.util.RandomFile;
import edu.arizona.cs.learn.util.Utils;

// Some things do not change
//    The number of examples... 60
//    The number of folds....   5

public class SyntheticExperiments {
	public static final int FOLDS = 6;
	public static final int N = 60;
	
	public static void main(String[] args) throws Exception { 
		Utils.LIMIT_RELATIONS = true;
		Utils.WINDOW = 2;
		
//		String pid = RandomFile.getPID();
//		generateClass(pid, "f", 1, 0, 25);
//		generateClass(pid, "g", 2, 0.1, 25);
		
		if (args.length != 5) {
			System.out.print("Usage: numRepeat experiment ");
			System.out.print("\"mean1,mean2,...\" \"pct1,pct2,..\"");
			System.out.print("\"length1,length2,...\"");
			System.out.println();
			return;
		}
		
		Utils.EXPERIMENTS = Integer.parseInt(args[0]);
		String experiment = args[1];
		System.out.println("Experiment: " + experiment);
		
		List<Double> means = new ArrayList<Double>();
		String[] meanTokens = args[2].split("[,]");
		for (String tok : meanTokens)
			means.add(Double.parseDouble(tok));
		
		List<Double> pcts = new ArrayList<Double>();
		String[] pctTokens = args[3].split("[,]");
		for (String tok : pctTokens)
			pcts.add(Double.parseDouble(tok));
		
		List<Integer> lengths = new ArrayList<Integer>();
		String[] lengthTokens = args[4].split("[,]");
		for (String tok : lengthTokens)
			lengths.add(Integer.parseInt(tok));
		
		if ("curve".equals(experiment)) 
			learningCurve(lengths, means, pcts);
		else if ("cluster".equals(experiment))
			cluster(lengths, means, pcts);
		else if ("knn".equals(experiment))
			knn(lengths, means, pcts);
		else
			experiment(lengths, means, pcts);
		
//		expectedSequenceSizes(lengths,means);
	}
		
	public static void learningCurve(List<Integer> lengths, List<Double> means, List<Double> pcts) throws Exception { 
		String pid = RandomFile.getPID();
		String dir = "/tmp/niall-" + pid + "/";
		
		LearningCurve curve = new LearningCurve();

		for (double pct : pcts) {
			for (double mean : means) { 
				for (int length : lengths) { 
					System.out.println("Mean: " + mean + " Pct: " + pct + " Length: " + length);

					generateClass(pid, "f", 0, 0, length);
					generateClass(pid, "g", mean, pct, length);

					Map<String,List<Instance>> data = Utils.load(dir, "niall", SequenceType.allen);
					curve.buildCurve("logs/niall-" + mean + "-" + pct + "-" + length, data);
				}
			}
		}
	}
	
	/**
	 * Class 1 does not change -- so we need to construct the niall-f.lisp file
	 * @param index - 1 for pure random class and 2 for the structured class
	 */
	public static void generateClass(String pid, String className, double mean, double pct, int eLength) { 		
		String prefix = "/tmp/niall-" + pid + "/";
		File f = new File(prefix);
		if (!f.exists())
			f.mkdir();
		
		try {
			int streams = 6;
			
			String cmd = "scripts/sim.R " + prefix + className + " " + streams + " " + eLength + " " + mean + " " + pct;
			System.out.println(cmd);
			Process p = Runtime.getRuntime().exec("Rscript " + cmd);
			p.waitFor();
		} catch (Exception e) { 
			e.printStackTrace();
		}
		
		SymbolicData.convert(prefix + className, prefix + "niall-" + className +".lisp", N);
	}

	/**
	 * Run clustering algorithm for all of the values across the means, lengths, and pct to vary
	 * the covariance between no covariance and an easy covariance structure.
	 * @param episodeLengths
	 * @param means
	 * @param pcts
	 * @throws Exception
	 */
	public static void cluster(List<Integer> episodeLengths, List<Double> means, List<Double> pcts) throws Exception { 
		String pid = RandomFile.getPID();
		String key = System.currentTimeMillis() + "";

		PrintStream out = new PrintStream(new File("logs/synthetic-kmeans-" + key + ".csv"));
		out.println("cluster_type,cluster_init,pct,mean,length,run,tp,fp,fn,tn,accuracy");
		for (double pct : pcts) { 
			for (double mean : means) { 
				for (int length : episodeLengths) { 
					generateClass(pid, "f", 0, 0, length);
					generateClass(pid, "g", mean, pct, length);
					
					Map<String,List<Instance>> data = Utils.load("/tmp/niall-" + pid + "/", "niall", SequenceType.allen);
					List<Instance> all = new ArrayList<Instance>();
					for (String name : data.keySet()) { 
						all.addAll(data.get(name));
					}
					
					// Now perform *all* of the experiments.
					for (ClusterInit init : ClusterInit.values()) { 
						for (ClusterType type : ClusterType.values()) { 
							
							// now we repeat the experiment x number of times....
							for (int i = 0; i < Utils.EXPERIMENTS; ++i) { 
								KMeans kmeans = new KMeans(data.keySet().size(), 20, init, type);
								ClusteringResults result = kmeans.cluster(all);
								out.println(type + "," + init + "," + pct + "," + mean + "," + length + "," + i + 
										"," + result.tp + "," + result.fp + "," + result.fn + "," + result.tn + 
										"," + result.accuracy);
							}
						}
					}
				}
			}
		}
	}	
	
	/**
	 * Run the for all of the values across the means, lengths, and pct to vary
	 * the covariance between no covariance and an easy covariance structure.
	 * @param episodeLengths
	 * @param means
	 * @param pcts
	 * @throws Exception
	 */
	public static void experiment(List<Integer> episodeLengths, List<Double> means, List<Double> pcts) throws Exception { 
		String pid = RandomFile.getPID();
		System.out.println("Means: " + means);
		System.out.println("Lengths: " + episodeLengths);
		System.out.println("Pcts: " + pcts);
		
		String key = System.currentTimeMillis() + "";
		
		BufferedWriter out = new BufferedWriter(new FileWriter("logs/synthetic-" + key + ".csv"));
		out.write("elength,mean,pct,test," + BatchStatistics.csvHeader() + "\n");
		
		// Total number of experiments equals 25
		for (double pct : pcts) { 
			for (double mean : means) { 
				for (int length : episodeLengths) { 
					System.out.println("Mean: " + mean + " Length: " + length);

					generateClass(pid, "f", 0, 0, length);
					generateClass(pid, "g", mean, pct, length);

					Map<String,List<Instance>> data = Utils.load("/tmp/niall-" + pid + "/", "niall", SequenceType.allen);
					List<String> classNames = new ArrayList<String>(data.keySet());
					Collections.sort(classNames);

					ClassifyParams params = new ClassifyParams();
					params.type = SequenceType.allen;
					params.prunePct = 0.5;
					Classifier c = Classify.prune.getClassifier(params);

					//				CrossValidation cv = new CrossValidation(FOLDS);
					SplitAndTest sat = new SplitAndTest(Utils.EXPERIMENTS, 2.0/3.0);
					List<BatchStatistics> stats = sat.run(System.currentTimeMillis(), classNames, data, c);

					// print out the summary information for now.  Later we will need to 
					// print out all of it.
					SummaryStatistics perf = new SummaryStatistics();
					SummaryStatistics[][] confMatrix = new SummaryStatistics[classNames.size()][classNames.size()];
					for (int i = 0; i < classNames.size(); ++i) 
						for (int j = 0; j < classNames.size(); ++j) 
							confMatrix[i][j] = new SummaryStatistics();

					// append to the file the results of this latest run...
					for (int i = 0; i < stats.size(); ++i) { 
						BatchStatistics batch = stats.get(i);
						out.write(batch.toCSV(length + "," + mean + "," + pct + "," + i + ",", ""));

						perf.addValue(batch.accuracy());
						double[][] matrix = batch.normalizeConfMatrix();
						for (int j = 0; j < classNames.size(); ++j)
							for (int k = 0; k < classNames.size(); ++k)
								confMatrix[j][k].addValue(matrix[j][k]);
					}
					out.flush();
					System.out.println("[pct:" + pct + ",mean:" + mean + ",length:" + length + "] " +
							"performance: " + perf.getMean() + " sd -- " + perf.getStandardDeviation());


					// Write out the confusion matrix for this pairing of variables.
					BufferedWriter outMatrix = new BufferedWriter(new FileWriter("logs/matrix-" + key + "-" + length + "-" + mean + ".csv"));
					for (int i = 0; i < classNames.size(); ++i) 
						outMatrix.write("," + classNames.get(i));
					outMatrix.write("\n");

					for (int i = 0; i < classNames.size(); ++i) {
						outMatrix.write(classNames.get(i));
						for (int j = 0; j < classNames.size(); ++j) 
							outMatrix.write("," + confMatrix[i][j].getMean());
						outMatrix.write("\n");
					}
					outMatrix.close();
				}
			}
		}
	}	
	
	/**
	 * Run the for all of the values across the means, lengths, and pct to vary
	 * the covariance between no covariance and an easy covariance structure.
	 * @param episodeLengths
	 * @param means
	 * @param pcts
	 * @throws Exception
	 */
	public static void knn(List<Integer> episodeLengths, List<Double> means, List<Double> pcts) throws Exception { 
		String pid = RandomFile.getPID();
		System.out.println("Means: " + means);
		System.out.println("Lengths: " + episodeLengths);
		System.out.println("Pcts: " + pcts);
		
		String key = System.currentTimeMillis() + "";
		
		BufferedWriter out = new BufferedWriter(new FileWriter("logs/synthetic-knn-" + key + ".csv"));
		out.write("classifier,elength,mean,pct,test," + BatchStatistics.csvHeader() + "\n");
		
		for (double pct : pcts) { 
			for (double mean : means) { 
				for (int length : episodeLengths) { 
					System.out.println("Mean: " + mean + " Length: " + length);

					generateClass(pid, "f", 0, 0, length);
					generateClass(pid, "g", mean, pct, length);

					Map<String,List<Instance>> data = Utils.load("/tmp/niall-" + pid + "/", "niall", SequenceType.allen);
					List<String> classNames = new ArrayList<String>(data.keySet());
					Collections.sort(classNames);

					int[] ks = { 1, 10 };
					for (int k : ks) { 
						ClassifyParams params = new ClassifyParams();
						params.type = SequenceType.allen;
						params.k = k;
						params.weighted = true;
						Classifier c = Classify.knn.getClassifier(params);

						SplitAndTest sat = new SplitAndTest(Utils.EXPERIMENTS, 2.0/3.0);
						List<BatchStatistics> stats = sat.run(System.currentTimeMillis(), classNames, data, c);

						SummaryStatistics perf = new SummaryStatistics();
						// append to the file the results of this latest run...
						for (int i = 0; i < stats.size(); ++i) { 
							BatchStatistics batch = stats.get(i);
							out.write(batch.toCSV("knn" + k + "," + length + "," + mean + "," + pct + "," + i + ",", ""));

							perf.addValue(batch.accuracy());
						}
						out.flush();
						System.out.println("[pct:" + pct + ",mean:" + mean + ",length:" + length + "] " +
								"performance: " + perf.getMean() + " sd -- " + perf.getStandardDeviation());
					}
				}
			}
		}
	}	

	
	public static void expectedSequenceSizes(List<Integer> episodeLengths, List<Double> means) throws Exception { 
		String pid = RandomFile.getPID();
		System.out.println("Means: " + means);
		System.out.println("Lengths: " + episodeLengths);

		String key = "" + System.currentTimeMillis();
		
		BufferedWriter out = new BufferedWriter(new FileWriter("logs/synthetic-sizes-" + key + ".csv"));
		out.write("elength,mean,test,actual_class,avgTrainingSize\n");

		for (double mean : means) { 
			for (int length : episodeLengths) { 
				System.out.println("Mean: " + mean + " Length: " + length);
				
				generateClass(pid, "f", 1, 0, length);
				generateClass(pid, "g", 2, mean, length);

				Map<String,List<Instance>> data = Utils.load("/tmp/niall-" + pid + "/", "niall", SequenceType.allen);
				List<String> classNames = new ArrayList<String>(data.keySet());
				Collections.sort(classNames);
				
				Random r = new Random(System.currentTimeMillis());
				for (int k = 0; k < 100; ++k) { 
					// perform the shuffle and split, but without actually learning
					// anything.  I just want the average length of the sequences
					for (String className : data.keySet()) { 
						SummaryStatistics ss = new SummaryStatistics();
						
						List<Instance> episodes = data.get(className);
						Collections.shuffle(episodes, r);

						int split = (int) Math.floor((2.0/3.0)* (double) episodes.size());
						for (int i = 0; i < split; ++i) {
							ss.addValue(episodes.get(i).sequence().size());
							
						}
						out.write(length + "," + mean + "," + k + "," + className + "," + ss.getMean() + "\n");
					}
					
				}
				
			}
		}
		out.close();
	}
}
