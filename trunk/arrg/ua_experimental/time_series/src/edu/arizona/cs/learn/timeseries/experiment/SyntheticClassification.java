package edu.arizona.cs.learn.timeseries.experiment;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Map;
import java.util.Random;

import org.apache.commons.math.stat.descriptive.SummaryStatistics;

import edu.arizona.cs.learn.timeseries.classification.Classifier;
import edu.arizona.cs.learn.timeseries.classification.Classify;
import edu.arizona.cs.learn.timeseries.classification.ClassifyParams;
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

public class SyntheticClassification {
	public static final int FOLDS = 6;
	public static final int N = 60;
	
	public static int EXPERIMENTS = 50;

	public static void main(String[] args) throws Exception { 
		Utils.LIMIT_RELATIONS = true;
		Utils.WINDOW = 2;
		
//		String pid = RandomFile.getPID();
//		generateClass(pid, "f", 1, 0, 25);
//		generateClass(pid, "g", 2, 0.1, 25);
		
		if (args.length != 4) {
			System.out.println("Usage: numRepeat experiment \"mean1,mean2,...\" \"length1,length2,...\"");
			return;
		}
		
		EXPERIMENTS = Integer.parseInt(args[0]);
		String experiment = args[1];
		
		List<Double> means = new ArrayList<Double>();
		String[] meanTokens = args[2].split("[,]");
		for (String tok : meanTokens)
			means.add(Double.parseDouble(tok));
		
		List<Integer> lengths = new ArrayList<Integer>();
		String[] lengthTokens = args[3].split("[,]");
		for (String tok : lengthTokens)
			lengths.add(Integer.parseInt(tok));
		
		if ("curve".equals(experiment)) 
			learningCurve(means, lengths);
		else
			experiment1(lengths, means);
//		expectedSequenceSizes(lengths,means);
	}
	
	public static void learningCurve(List<Double> means, List<Integer> episodeLengths) throws Exception { 
		String pid = RandomFile.getPID();
		String dir = "/tmp/niall-" + pid + "/";
		
		LearningCurve curve = new LearningCurve();

		for (double mean : means) { 
			for (int length : episodeLengths) { 
		
				generateClass(pid, "f", 1, 0, length);
				generateClass(pid, "g", 2, mean, length);
		
				Map<String,List<Instance>> data = Utils.load(dir, "niall", SequenceType.allen);
				curve.buildCurve("logs/niall-" + mean + "-" + length, data);
			}
		}
	}
	
	/**
	 * Class 1 does not change -- so we need to construct the niall-f.lisp file
	 * @param index - 1 for pure random class and 2 for the structured class
	 */
	public static void generateClass(String pid, String className, int index, double mean, int eLength) { 		
		String prefix = "/tmp/niall-" + pid + "/";
		File f = new File(prefix);
		if (!f.exists())
			f.mkdir();
		try {

			
			// initial arguments
			//   prefix - where you want the file written
			//   p - the number of streams.
			//   alphabet.size - the number of symbols (constant in each stream)
			//   episode.length -- the length of each episode
			int streams = 6;
			
			String cmd = "scripts/sim.R " + index + " " + prefix + " " + streams + " " + mean + " " + eLength;
			System.out.println(cmd);
			Process p = Runtime.getRuntime().exec("Rscript " + cmd);
			p.waitFor();
		} catch (Exception e) { 
			e.printStackTrace();
		}
		
		SymbolicData.convert(prefix + className, prefix + "niall-" + className +".lisp", N);
	}
	
	public static void experiment1(List<Integer> episodeLengths, List<Double> means) throws Exception { 
//		int[] episodeLengths = new int[] { 200 }; //, 200, 300, 400, 500 };
//		double[] means = new double[] { 0.00 }; //, 0.5, 1.0, 1.5, 2.0 };
		String pid = RandomFile.getPID();
		System.out.println("Means: " + means);
		System.out.println("Lengths: " + episodeLengths);
		
		String key = System.currentTimeMillis() + "";
		
		BufferedWriter out = new BufferedWriter(new FileWriter("logs/synthetic-" + key + ".csv"));
		out.write("elength,mean,test," + BatchStatistics.csvHeader() + "\n");
		
		// Total number of experiments equals 25
		for (double mean : means) { 
			for (int length : episodeLengths) { 
				System.out.println("Mean: " + mean + " Length: " + length);
				
				generateClass(pid, "f", 1, 0, length);
				generateClass(pid, "g", 2, mean, length);

				Map<String,List<Instance>> data = Utils.load("/tmp/niall-" + pid + "/", "niall", SequenceType.allen);
				List<String> classNames = new ArrayList<String>(data.keySet());
				Collections.sort(classNames);
				
				ClassifyParams params = new ClassifyParams();
				params.type = SequenceType.allen;
				params.prunePct = 0.5;
				Classifier c = Classify.prune.getClassifier(params);
				
//				CrossValidation cv = new CrossValidation(FOLDS);
				SplitAndTest sat = new SplitAndTest(EXPERIMENTS, 2.0/3.0);
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
					out.write(batch.toCSV(length + "," + mean + "," + i + ",", ""));

					perf.addValue(batch.accuracy());
					double[][] matrix = batch.normalizeConfMatrix();
					for (int j = 0; j < classNames.size(); ++j)
						for (int k = 0; k < classNames.size(); ++k)
							confMatrix[j][k].addValue(matrix[j][k]);
				}
				out.flush();
				System.out.println("[" + mean + "," + length + "] performance: " + perf.getMean() + " sd -- " + perf.getStandardDeviation());

				
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
