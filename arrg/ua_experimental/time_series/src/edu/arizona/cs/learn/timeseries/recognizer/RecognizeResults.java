package edu.arizona.cs.learn.timeseries.recognizer;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

import edu.arizona.cs.learn.algorithm.markov.FSMRecognizer;

public class RecognizeResults {
	public Map<String, Boolean> recognized;
	public Map<String, Integer> timeStep;
	
	public String testClass;
	public int testId;

	public RecognizeResults(List<FSMRecognizer> recognizers, String testClass, int testId) {
		this.testClass = testClass;
		this.testId = testId;

		this.recognized = new HashMap<String,Boolean>();
		this.timeStep = new HashMap<String,Integer>();

		for (FSMRecognizer r : recognizers) {
			this.recognized.put(r.key(), Boolean.valueOf(false));
			this.timeStep.put(r.key(), Integer.valueOf(-1));
		}
	}
}