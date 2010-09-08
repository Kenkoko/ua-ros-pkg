package edu.arizona.cs.learn.timeseries.classification;

import edu.arizona.cs.learn.algorithm.alignment.model.Instance;

public class ClassifyResults {
	public String className;
	public Classifier classifier;
	public Instance test;
	public double elapsed;

	public ClassifyResults(Classifier c, Instance test, String className, double elapsed) {
		this.className = className;
		this.classifier = c;
		this.test = test;
		this.elapsed = elapsed;
	}
}