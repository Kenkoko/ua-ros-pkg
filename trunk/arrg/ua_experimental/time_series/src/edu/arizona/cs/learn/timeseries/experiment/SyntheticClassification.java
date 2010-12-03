package edu.arizona.cs.learn.timeseries.experiment;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Map;
import java.util.Random;
import java.util.TreeMap;

import org.apache.commons.math.stat.descriptive.SummaryStatistics;

import edu.arizona.cs.learn.algorithm.alignment.model.Instance;
import edu.arizona.cs.learn.timeseries.Experiments;
import edu.arizona.cs.learn.timeseries.classification.Classifier;
import edu.arizona.cs.learn.timeseries.classification.Classify;
import edu.arizona.cs.learn.timeseries.evaluation.BatchStatistics;
import edu.arizona.cs.learn.timeseries.evaluation.CrossValidation;
import edu.arizona.cs.learn.timeseries.prep.SymbolicData;
import edu.arizona.cs.learn.util.SequenceType;
import edu.arizona.cs.learn.util.Utils;

// Some things do not change
//    The number of examples... 60
//    The number of folds....   5

public class SyntheticClassification {
	public static final int FOLDS = 6;
	public static final int N = 60;

	public static void main(String[] args) throws Exception { 
		Utils.LIMIT_RELATIONS = true;
		Utils.WINDOW = 2;
		
		if (args.length != 2) 
			throw new RuntimeException("Usage: \"mean1,mean2,...\" \"length1,length2,...\"");
//		experiment1();
		
		List<Double> means = new ArrayList<Double>();
		String[] meanTokens = args[0].split("[,]");
		for (String tok : meanTokens)
			means.add(Double.parseDouble(tok));
		
		List<Integer> lengths = new ArrayList<Integer>();
		String[] lengthTokens = args[1].split("[,]");
		for (String tok : lengthTokens)
			lengths.add(Integer.parseInt(tok));
		
		experiment1(lengths, means);
	}
	
	/**
	 * Called once... it sets up the cross-validation
	 * folders and randomly shuffles the episodes into
	 * test sets.
	 * 
	 * After that it generates all of the examples for
	 * class 1
	 */
	public static void initialize() { 
		Random r = new Random(System.currentTimeMillis());
		
		// Ensure that some directories exist
		File crossDir = new File("data/cross-validation/");
		if (!crossDir.exists())
			crossDir.mkdir();
		
		File kDir = new File("data/cross-validation/k" + FOLDS + "/");
		if (!kDir.exists())
			kDir.mkdir();

		// two classes - niall-f and niall-g
		List<String> classNames = new ArrayList<String>();
		classNames.add("niall-f");
		classNames.add("niall-g");
		List<Map<String,List<Integer>>> sets = new ArrayList<Map<String,List<Integer>>>();
		for (int i = 0; i < FOLDS; i++) {
			Map<String,List<Integer>> map = new TreeMap<String,List<Integer>>();
			for (String c : classNames) 
				map.put(c, new ArrayList<Integer>());
			sets.add(map);
		}

		// Randomly select some of the instances
		// for each of the folds.
		for (String className : classNames) { 
			List<Integer> episodes = new ArrayList<Integer>();
			for (int i = 1; i <= N; ++i) 
				episodes.add(i);
			Collections.shuffle(episodes, r);
			
			for (int i = 0; i < episodes.size(); ++i) {
				sets.get(i % FOLDS).get(className).add(episodes.get(i));
			}
		}

		Experiments.writeTestFile("niall", classNames, sets, FOLDS);
		
		// create the folder for signatures trained from sequences of
		// Allen relations.
		for (int i = 0; i < FOLDS; i++) {
			String f = "data/cross-validation/k" + FOLDS + "/fold-" + i + "/allen/";
				File file = new File(f);
				if (!file.exists())
					file.mkdir();
		}		
	}
	
	/**
	 * Class 1 does not change -- so we need to construct the niall-f.lisp file
	 * @param index - 1 for pure random class and 2 for the structured class
	 */
	public static void generateClass(String className, int index, double mean, int eLength) { 		
		try {
			File f = new File("/tmp/niall/");
			if (!f.exists())
				f.mkdir();

			
			// initial arguments
			//   prefix - where you want the file written
			//   p - the number of streams.
			//   alphabet.size - the number of symbols (constant in each stream)
			//   episode.length -- the length of each episode
			String prefix = "/tmp/niall/";
			int streams = 6;
			
			String cmd = "scripts/sim.R " + index + " " + prefix + " " + streams + " " + mean + " " + eLength;
			System.out.println(cmd);
			Process p = Runtime.getRuntime().exec("Rscript " + cmd);
			p.waitFor();
		} catch (Exception e) { 
			e.printStackTrace();
		}
		
		SymbolicData.convert("/tmp/niall/" + className, "data/input/niall-" + className +".lisp", N);
	}
	
	public static void experiment1(List<Integer> episodeLengths, List<Double> means) throws Exception { 
//		int[] episodeLengths = new int[] { 200 }; //, 200, 300, 400, 500 };
//		double[] means = new double[] { 0.00 }; //, 0.5, 1.0, 1.5, 2.0 };
		System.out.println("Means: " + means);
		System.out.println("Lengths: " + episodeLengths);
		
		String key = System.currentTimeMillis() + "";
		
		BufferedWriter out = new BufferedWriter(new FileWriter("logs/synthetic-" + key + ".csv"));
		out.write("elength,mean," + BatchStatistics.csvHeader() + "\n");
		
		// Total number of experiments equals 25
		for (double mean : means) { 
			for (int length : episodeLengths) { 
				System.out.println("Mean: " + mean + " Length: " + length);
				
				generateClass("f", 1, 0, length);
				generateClass("g", 2, mean, length);

				Map<String,List<Instance>> data = Utils.load("niall", SequenceType.allen);
				List<String> classNames = new ArrayList<String>(data.keySet());
				Collections.sort(classNames);
				
				Classifier c = Classify.prune.getClassifier(SequenceType.allen, -1, 50, false, -1);
				
				CrossValidation cv = new CrossValidation(FOLDS);
//				SplitAndTest sat = new SplitAndTest(100, pct);
				List<BatchStatistics> stats = cv.run(System.currentTimeMillis(), classNames, data, c);

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
					out.write(batch.toCSV(length + "," + mean + ",", ""));

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
}
