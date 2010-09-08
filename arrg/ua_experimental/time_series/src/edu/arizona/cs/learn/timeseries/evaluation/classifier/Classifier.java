package edu.arizona.cs.learn.timeseries.evaluation.classifier;

import java.util.List;

import edu.arizona.cs.learn.algorithm.alignment.model.Instance;
import edu.arizona.cs.learn.timeseries.evaluation.FoldStatistics;

public interface Classifier {
	
	public String getName();

	public void addData(FoldStatistics fold);
	public void train(List<List<Instance>> data, int x);
	public void train(List<Instance> trainingData);
	
	public String test(Instance instance);
	
	public void printError(Instance instance, int id);
}
