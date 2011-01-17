package edu.arizona.cs.learn.timeseries.evaluation;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Future;

import org.apache.commons.math.stat.descriptive.SummaryStatistics;

import edu.arizona.cs.learn.timeseries.classification.Classifier;
import edu.arizona.cs.learn.timeseries.classification.ClassifyCallable;
import edu.arizona.cs.learn.timeseries.model.Instance;

public abstract class ClassificationTest {
	
	protected ExecutorService _execute;

	public BatchStatistics runBatch(int batch, 
			Classifier c, List<String> classNames, 
			Map<String,List<Instance>> train, Map<String,List<Instance>> test) { 
		BatchStatistics fs = new BatchStatistics(c.getName(), classNames);

		// Split them into groups by class name.
		Map<String,SummaryStatistics> sizeMap = new HashMap<String,SummaryStatistics>();
		for (String key : train.keySet()) { 
			SummaryStatistics size = new SummaryStatistics();
			sizeMap.put(key, size);
			
			for (Instance instance : train.get(key)) 
				size.addValue(instance.sequence().size());
		}

		Map<String,Long> timing = c.train(batch, train);

		// Add the training data to the batch statistics...
		for (String key : timing.keySet()) { 
			fs.addTrainingDetail(key, train.get(key).size(), timing.get(key), sizeMap.get(key).getMean());
		}
		

		List<Future<ClassifyCallable>> futureList = new ArrayList<Future<ClassifyCallable>>();
		for (List<Instance> instances : test.values())
			for (Instance instance : instances) 
				futureList.add(_execute.submit(new ClassifyCallable(c, instance)));
		
		for (Future<ClassifyCallable> future : futureList) {
			try {
				ClassifyCallable results = future.get();
				fs.addTestDetail(results.actual(), results.predicted(), results.duration());
			} catch (Exception e) {
				e.printStackTrace();
			} 
		}	
		return fs;
	}
}
