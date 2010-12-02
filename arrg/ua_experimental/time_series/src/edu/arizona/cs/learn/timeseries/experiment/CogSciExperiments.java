package edu.arizona.cs.learn.timeseries.experiment;

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Map;

import org.apache.commons.math.stat.descriptive.SummaryStatistics;

import edu.arizona.cs.learn.algorithm.alignment.model.Instance;
import edu.arizona.cs.learn.timeseries.classification.Classifier;
import edu.arizona.cs.learn.timeseries.classification.Classify;
import edu.arizona.cs.learn.timeseries.evaluation.BatchStatistics;
import edu.arizona.cs.learn.timeseries.evaluation.CrossValidation;
import edu.arizona.cs.learn.timeseries.evaluation.SplitAndTest;
import edu.arizona.cs.learn.util.SequenceType;
import edu.arizona.cs.learn.util.Utils;

/**
 * These are the experiments that will be run
 * for the Cognitive Science Journal paper that
 * I am currently working on.  If useful, they 
 * will be extracted accordingly.
 * @author wkerr
 *
 */
public class CogSciExperiments {

	/**
	 * Calculate the performance of the CAVE classifier on
	 * the prefix given
	 * @param prefix
	 */
	public static void performance(String prefix) { 
		Map<String,List<Instance>> data = Utils.load(prefix, SequenceType.allen);
		List<String> classNames = new ArrayList<String>(data.keySet());
		Collections.sort(classNames);
		
		Classifier c = Classify.prune.getClassifier(SequenceType.allen, -1, 50, false, -1);
		
		CrossValidation cv = new CrossValidation(5);
		List<BatchStatistics> stats = cv.run(System.currentTimeMillis(), classNames, data, c);
		
		// For now let's print out some informative stuff and build
		// one confusion matrix
		SummaryStatistics perf = new SummaryStatistics();
		int[][] matrix = new int[classNames.size()][classNames.size()];
		for (int i = 0; i < stats.size(); ++i) { 
			BatchStatistics fs = stats.get(i);
			double accuracy = fs.accuracy();
			int[][] confMatrix = fs.confMatrix();

			perf.addValue(accuracy);
			System.out.println("Fold - " + i + " -- " + accuracy);
			
			for (int j = 0; j < classNames.size(); ++j) { 
				for (int k = 0; k < classNames.size(); ++k) { 
					matrix[j][k] += confMatrix[j][k];
				}
			}
		}
		System.out.println("Performance: " + perf.getMean() + " sd -- " + perf.getStandardDeviation());
		
		// Now print out the confusion matrix (in csv format)
		System.out.println(Utils.toCSV(classNames, matrix));
	}
	
	/**
	 * Generate a learning curve by splitting and testing for
	 * multiple percents of training data.
	 */
	public static void learningCurve(String prefix) throws Exception { 
		Map<String,List<Instance>> data = Utils.load(prefix, SequenceType.allen);
		List<String> classNames = new ArrayList<String>(data.keySet());
		Collections.sort(classNames);
		
		Classifier c = Classify.prune.getClassifier(SequenceType.allen, -1, 50, false, -1);
		
		// Construct a file to save the results to...
		BufferedWriter out = new BufferedWriter(new FileWriter("logs/" + prefix + "-learning-curve.csv"));
		
		// write out the header...
		out.write("training_pct,batch," + BatchStatistics.csvHeader() + "\n");
		
		double[] pcts = new double[] { 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8 };
		for (double pct : pcts) { 
			SplitAndTest sat = new SplitAndTest(5, pct);
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
				out.write(batch.toCSV((int)(pct*100) + "," + i + ",", ""));

				perf.addValue(batch.accuracy());
				double[][] matrix = batch.normalizeConfMatrix();
				for (int j = 0; j < classNames.size(); ++j)
					for (int k = 0; k < classNames.size(); ++k)
						confMatrix[j][k].addValue(matrix[j][k]);
			}
			System.out.println("[" + pct + "] Performance: " + perf.getMean() + " sd -- " + perf.getStandardDeviation());
			System.out.println("  Matrix");

			for (int i = 0; i < classNames.size(); ++i) 
				System.out.print("," + classNames.get(i));
			System.out.println();
			
			for (int i = 0; i < classNames.size(); ++i) {
				System.out.print(classNames.get(i));
				for (int j = 0; j < classNames.size(); ++j) 
					System.out.print("," + confMatrix[i][j].getMean());
				System.out.println();
			}
			out.flush();
		}
		
		out.close();
	}
	
	public static void main(String[] args) throws Exception { 
//		performance("ww3d");
//		learningCurve("ww3d");
		learningCurve("ww2d");
	}
	
}
