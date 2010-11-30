package edu.arizona.cs.learn.timeseries.evaluation;

import java.util.ArrayList;
import java.util.List;


public class BatchStatistics {
	public String classifierName;
	
	public List<String> actualClass;
	public List<String> predictedClass;
	public List<Boolean> detail;

	public double trainingTime;
	public double testingTime;
		
	public int[][] confMatrix;
	
	public BatchStatistics(String classifierName, int nClasses) { 
		this.classifierName = classifierName;
		
		confMatrix = new int[nClasses][nClasses];
		detail = new ArrayList<Boolean>();
		actualClass = new ArrayList<String>();
		predictedClass = new ArrayList<String>();
	}
	
	public double accuracy() { 
		double right = 0;
		for (Boolean b : detail) { 
			if (b) ++right;
		}
		
		return right / (double) detail.size();
	}
}
