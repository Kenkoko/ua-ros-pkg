package edu.arizona.cs.learn.timeseries.experiment;

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
			
			perf.addValue(accuracy);
			System.out.println("Fold - " + i + " -- " + accuracy);
			
			for (int j = 0; j < classNames.size(); ++j) { 
				for (int k = 0; k < classNames.size(); ++k) { 
					matrix[j][k] += fs.confMatrix[j][k];
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
	public static void learningCurve(String prefix) { 
		Map<String,List<Instance>> data = Utils.load(prefix, SequenceType.allen);
		List<String> classNames = new ArrayList<String>(data.keySet());
		Collections.sort(classNames);
		
		Classifier c = Classify.prune.getClassifier(SequenceType.allen, -1, 50, false, -1);
		
		double[] pcts = new double[] { 0.1, 0.2, 0.3, 0.4, 0.5, 0.6 };
		for (double pct : pcts) { 
			SplitAndTest sat = new SplitAndTest(50, pct);
			List<BatchStatistics> stats = sat.run(System.currentTimeMillis(), classNames, data, c);
			
			// print out the summary information for now.  Later we will need to 
			// print out all of it.
			SummaryStatistics perf = new SummaryStatistics();
			for (int i = 0; i < stats.size(); ++i) 
				perf.addValue(stats.get(i).accuracy());
			System.out.println("[" + pct + "] Performance: " + perf.getMean() + " sd -- " + perf.getStandardDeviation());
		}
	}
	
	public static void main(String[] args) { 
//		performance("ww3d");
		learningCurve("ww3d");
	}
	
}
