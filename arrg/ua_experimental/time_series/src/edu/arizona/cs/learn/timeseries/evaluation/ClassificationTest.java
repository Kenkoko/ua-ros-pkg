package edu.arizona.cs.learn.timeseries.evaluation;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Future;

import edu.arizona.cs.learn.algorithm.alignment.model.Instance;
import edu.arizona.cs.learn.timeseries.classification.Classifier;
import edu.arizona.cs.learn.timeseries.classification.ClassifyCallable;
import edu.arizona.cs.learn.timeseries.classification.ClassifyResults;

public abstract class ClassificationTest {
	
	protected ExecutorService _execute;

	public BatchStatistics runBatch(int batch, 
			Classifier c, List<String> classNames, 
			List<Instance> train, List<Instance> test) { 
		BatchStatistics fs = new BatchStatistics(c.getName(), classNames.size());
		
		long startTime = System.currentTimeMillis();
		c.train(batch, train);
		fs.trainingTime = System.currentTimeMillis() - startTime;

		List<Future<ClassifyResults>> futureList = new ArrayList<Future<ClassifyResults>>();
		for (Instance instance : test)  
			futureList.add(_execute.submit(new ClassifyCallable(c, instance)));
		
		for (Future<ClassifyResults> future : futureList) {
			ClassifyResults results = null;
			try {
				results = future.get();
			} catch (Exception e) {
				e.printStackTrace();
			} 

			fs.testingTime += results.elapsed;

			int correctIndex = classNames.indexOf(results.test.name());
			int classifyIndex = classNames.indexOf(results.className);
			fs.confMatrix[correctIndex][classifyIndex] += 1;
			
			if (results.test.name().equals(results.className))  
				fs.detail.add(true);
			else  
				fs.detail.add(false);
		}	
		return fs;
	}
}
