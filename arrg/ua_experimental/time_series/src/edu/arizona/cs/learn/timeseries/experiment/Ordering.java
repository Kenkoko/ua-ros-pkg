package edu.arizona.cs.learn.timeseries.experiment;

import java.io.File;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Random;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;

import org.apache.commons.math.stat.descriptive.SummaryStatistics;

import edu.arizona.cs.learn.timeseries.classification.CAVEClassifier;
import edu.arizona.cs.learn.timeseries.classification.Classifier;
import edu.arizona.cs.learn.timeseries.classification.ClassifyCallable;
import edu.arizona.cs.learn.timeseries.classification.ClassifyParams;
import edu.arizona.cs.learn.timeseries.model.Instance;
import edu.arizona.cs.learn.timeseries.model.Interval;
import edu.arizona.cs.learn.timeseries.model.SequenceType;
import edu.arizona.cs.learn.util.Utils;

public class Ordering {

	private ExecutorService _execute;

	private List<String> _classNames;
	private Map<String,List<Integer>> _testMap;
	
	private List<Instance> _testing;
	private Map<String,List<Instance>> _training;
	
	public Ordering() { 

	}
	
	/**
	 * Determine the list of test instances and the list of 
	 * training instances.
	 * @param prefix
	 * @param type
	 * @param prune
	 */
	public void prepare(String prefix, SequenceType type) { 
		_classNames = Utils.getActivityNames(prefix);
		_testMap = getTestSet(_classNames);
		
		// build a training set and a test set.
		_training = new HashMap<String,List<Instance>>();
		_testing = new ArrayList<Instance>();

		// Now train up the signatures...
		for (String className : _classNames) { 
			File dataFile = new File("data/input/" + className + ".lisp");
			List<Instance> instances = Utils.sequences(className, dataFile.getAbsolutePath(), type);
			List<Integer> testSet = _testMap.get(className);
			
			for (Instance instance : instances) { 
				if (testSet.contains(instance.id()))
					_testing.add(instance);
				else { 
					List<Instance> list = _training.get(instance.name());
					if (list == null) { 
						list = new ArrayList<Instance>();
						_training.put(instance.name(), list);
					}
					list.add(instance);
				}
			}
		}
	}
	
	public double experiment(SequenceType type, boolean prune) { 
		for (List<Instance> list : _training.values()) 
			Collections.shuffle(list, new Random(System.currentTimeMillis()));
		ClassifyParams params = new ClassifyParams();
		params.type = type;
		params.prunePct = 0.5;
		params.incPrune = prune;
		params.folds = 2;
		
		Classifier c = new CAVEClassifier(params);
		c.train(0, _training);
		
		return evaluate(c);
	}
	
	/**
	 * See if ordering makes any difference on the classifiers.  First I'll look at 
	 * 100 samples
	 * @param prefix
	 * @param type
	 * @param prune
	 */
	public double orderingExperiment(String prefix, SequenceType type, boolean prune) { 
		prepare(prefix, type);
		
		ClassifyParams params = new ClassifyParams();
		params.type = type;
		params.prunePct = 0.5;
		params.incPrune = true;
		params.folds = 2;
		Classifier c = new CAVEClassifier(params);
		c.train(0, _training);
		
		return evaluate(c);
	}

	
	public double evaluate(Classifier c) { 
		_execute = Executors.newFixedThreadPool(Utils.numThreads);
		List<Future<ClassifyCallable>> future = new ArrayList<Future<ClassifyCallable>>();

		for (Instance instance : _testing) {
			future.add(_execute.submit(new ClassifyCallable(c, instance)));
		}

		
		double correct = 0;
		for (Future<ClassifyCallable> results : future) {
			try {
				ClassifyCallable callable = results.get();
				if (callable.actual().equals(callable.predicted()))
					correct += 1;
			} catch (Exception e) {
				e.printStackTrace();
			} 
		}

		double accuracy = correct / (double) _testing.size();
		_execute.shutdown();
		return accuracy;
	}
	

	/**
	 * Select a random set to be the test set.
	 * @param classNames
	 * @return
	 */
	public Map<String,List<Integer>> getTestSet(List<String> classNames) { 
		Random r = new Random(System.currentTimeMillis());
		
		Map<String,List<Integer>> testSet = new HashMap<String,List<Integer>>();

		for (String className : classNames) { 
			String f = "data/input/" + className + ".lisp";
			Map<Integer,List<Interval>> map = Utils.load(new File(f));
			List<Integer> episodes = new ArrayList<Integer>(map.keySet());
			Collections.shuffle(episodes, r);
			
			// 33% of the instances will be part of the test set
			double pct = 1.0/3.0;
			int number = (int) Math.round((double) episodes.size() * pct);

			List<Integer> list = new ArrayList<Integer>();
			for (int i = 0; i < number; ++i) { 
				list.add(episodes.get(i));
			}
			testSet.put(className, list);
		}
		
		return testSet;
	}
	
	public static void main(String[] args) { 
		Ordering ordering = new Ordering();
		ordering.prepare("ww3d", SequenceType.allen);
		
		SummaryStatistics ss = new SummaryStatistics();
		for (int i = 0; i < 100; ++i) { 
			double value = ordering.experiment(SequenceType.allen, true);
//			double value = ordering.orderingExperiment("ww3d", SequenceType.allen, true);
			System.out.println("..." + value);
			ss.addValue(value);
		}
		System.out.println("Summary " + ss.getMean() + " -- " + ss.getStandardDeviation());
	}
}
