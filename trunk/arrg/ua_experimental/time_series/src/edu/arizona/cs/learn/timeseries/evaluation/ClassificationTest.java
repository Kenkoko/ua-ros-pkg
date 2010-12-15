package edu.arizona.cs.learn.timeseries.evaluation;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Future;

import org.apache.commons.math.stat.descriptive.SummaryStatistics;

import edu.arizona.cs.learn.algorithm.alignment.model.Instance;
import edu.arizona.cs.learn.timeseries.classification.Classifier;
import edu.arizona.cs.learn.timeseries.classification.ClassifyCallable;

public abstract class ClassificationTest {
	
	protected ExecutorService _execute;

	public BatchStatistics runBatch(int batch, 
			Classifier c, List<String> classNames, 
			List<Instance> train, List<Instance> test) { 
		BatchStatistics fs = new BatchStatistics(c.getName(), classNames);

		// Split them into groups by class name.
		Map<String,SummaryStatistics> sizeMap = new HashMap<String,SummaryStatistics>();
		Map<String,List<Instance>> instances = new HashMap<String,List<Instance>>();
		for (Instance instance : train) {
			List<Instance> list = instances.get(instance.name());
			if (list == null) {
				list = new ArrayList<Instance>();
				instances.put(instance.name(), list);
				
				SummaryStatistics size = new SummaryStatistics();
				sizeMap.put(instance.name(), size);
			}
			list.add(instance);
			sizeMap.get(instance.name()).addValue(instance.sequence().size());
		}

		Map<String,Long> timing = c.train(batch, instances);

		// Add the training data to the batch statistics...
		for (String key : timing.keySet()) { 
			fs.addTrainingDetail(key, instances.get(key).size(), timing.get(key), sizeMap.get(key).getMean());
		}
		

		List<Future<ClassifyCallable>> futureList = new ArrayList<Future<ClassifyCallable>>();
		for (Instance instance : test)  
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
