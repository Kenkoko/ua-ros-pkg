package edu.arizona.cs.learn.timeseries.classification;

import java.util.concurrent.Callable;

import edu.arizona.cs.learn.timeseries.model.Instance;

public class ClassifyCallable implements Callable<ClassifyCallable> {
	
	private Classifier _classifier;
	private Instance _testInstance;

	private String _predicted;
	private long _duration;

	public ClassifyCallable(Classifier c, Instance test) {
		_classifier = c;
		_testInstance = test;
	}

	public ClassifyCallable call() throws Exception {
		long start = System.currentTimeMillis();
		_predicted = _classifier.test(_testInstance);
		_duration = System.currentTimeMillis() - start;
		return this;
	}
	
	public String predicted() { 
		return _predicted;
	}
	
	public String actual() { 
		return _testInstance.name();
	}
	
	public Long duration() { 
		return _duration;
	}
}