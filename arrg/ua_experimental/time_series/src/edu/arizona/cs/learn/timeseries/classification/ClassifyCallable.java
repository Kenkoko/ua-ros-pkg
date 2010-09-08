package edu.arizona.cs.learn.timeseries.classification;

import java.util.concurrent.Callable;

import edu.arizona.cs.learn.algorithm.alignment.model.Instance;

public class ClassifyCallable implements Callable<ClassifyResults>
{
	public String className;
	public Classifier classifier;
	public Instance test;

	public ClassifyCallable(Classifier c, Instance test) {
		this.classifier = c;
		this.test = test;
	}

	public ClassifyResults call() throws Exception {
		long testStart = System.currentTimeMillis();
		this.className = this.classifier.test(this.test);
		long testEnd = System.currentTimeMillis();
		return new ClassifyResults(this.classifier, this.test, this.className, testEnd - testStart);
	}
}