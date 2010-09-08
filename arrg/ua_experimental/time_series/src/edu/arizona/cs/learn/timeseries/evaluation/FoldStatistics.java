package edu.arizona.cs.learn.timeseries.evaluation;

import org.apache.commons.math.stat.descriptive.SummaryStatistics;

public class FoldStatistics {

	public int foldSize;
	public double accuracy;
	
	public SummaryStatistics signatureSize;
	
	public SummaryStatistics testingLengths;
	public SummaryStatistics trainingLengths;
	
	public double trainingTime;
	public double testingTime;
	
	public FoldStatistics() { 
		testingLengths = new SummaryStatistics();
		trainingLengths = new SummaryStatistics();

		signatureSize = new SummaryStatistics();
	}
	
}
