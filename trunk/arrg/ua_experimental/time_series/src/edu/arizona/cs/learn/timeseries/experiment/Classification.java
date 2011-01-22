package edu.arizona.cs.learn.timeseries.experiment;

import java.io.File;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.TreeSet;

import org.apache.commons.math.stat.descriptive.SummaryStatistics;

import edu.arizona.cs.learn.algorithm.alignment.Similarity;
import edu.arizona.cs.learn.timeseries.classification.Classifier;
import edu.arizona.cs.learn.timeseries.classification.Classify;
import edu.arizona.cs.learn.timeseries.classification.ClassifyParams;
import edu.arizona.cs.learn.timeseries.evaluation.BatchSignatures;
import edu.arizona.cs.learn.timeseries.evaluation.BatchStatistics;
import edu.arizona.cs.learn.timeseries.evaluation.SplitAndTest;
import edu.arizona.cs.learn.timeseries.model.Instance;
import edu.arizona.cs.learn.timeseries.model.Interval;
import edu.arizona.cs.learn.timeseries.model.SequenceType;
import edu.arizona.cs.learn.timeseries.model.symbols.Symbol;
import edu.arizona.cs.learn.util.Utils;

public class Classification {

	/**
	 * Calculate the performance of the CAVE classifier on
	 * the files given
	 * @param data
	 */
	public static void performance(Map<String,List<Instance>> data) { 
		
		List<String> classNames = new ArrayList<String>(data.keySet());
		Collections.sort(classNames);
		
		ClassifyParams params = new ClassifyParams();
		params.prunePct = 0.5;
		params.incPrune = true;
		params.similarity = Similarity.cosine;
		Classifier c = Classify.prune.getClassifier(params);
		
		SplitAndTest sad = new SplitAndTest(10, 2.0/3.0);
//		CrossValidation cv = new CrossValidation(5);
		List<BatchStatistics> stats = sad.run(System.currentTimeMillis(), classNames, data, c);
		
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
	
	public static void lispPerformance(Map<String,List<Instance>> data) { 
		List<String> classNames = new ArrayList<String>(data.keySet());
		Collections.sort(classNames);
		
		ClassifyParams params = new ClassifyParams();
		params.prunePct = 0.5;
		params.incPrune = true;
		params.similarity = Similarity.strings;
		Classifier c = Classify.prune.getClassifier(params);
		
		SplitAndTest sad = new SplitAndTest(10, 2.0/3.0);
		List<BatchStatistics> stats = sad.run(System.currentTimeMillis(), classNames, data, c);
		
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
	
	public static void main(String[] args) throws Exception { 
//		Map<String,String> map = new HashMap<String,String>();
//		for (String character : Utils.alphabet) { 
//			map.put(character, "data/raw-data/handwriting/wes/xml/" + character + ".xml");
//		}
//		
//		Map<String,List<Instance>> data = new HashMap<String,List<Instance>>();
//		for (String key : map.keySet()) { 
//			data.put(key, XMLUtils.loadXML(map.get(key)));
//		}
//		
//		performance(data);
		
		lispPerformance(Utils.load("wes-pen", SequenceType.allen));
		
//		List<String> activities = Utils.getActivityNames("ww3d");
//		for (String s : activities) { 
//			map.put(s, "data/input/" + s + ".lisp");
//		}
//		
//		performance(Utils.convert(map));
		
//		performance("ww3d");
//		learningCurve("ww3d");
//		learningCurve("ww2d");
	}
}
