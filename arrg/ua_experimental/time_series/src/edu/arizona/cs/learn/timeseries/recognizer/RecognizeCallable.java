package edu.arizona.cs.learn.timeseries.recognizer;

import java.util.List;
import java.util.concurrent.Callable;

import edu.arizona.cs.learn.algorithm.markov.FSMRecognizer;
import edu.arizona.cs.learn.timeseries.model.Interval;

public class RecognizeCallable implements Callable<RecognizeResults> {
	public List<FSMRecognizer> recognizers;

	public String testClass;
	public int testId;
	
	public List<Interval> intervals;

	public RecognizeCallable(List<FSMRecognizer> recognizers, String testClass,
			int testId, List<Interval> intervals) {
		this.recognizers = recognizers;

		this.testClass = testClass;
		this.testId = testId;

		this.intervals = intervals;
	}

	public RecognizeResults call() throws Exception {
		RecognizeResults rr = new RecognizeResults(this.recognizers,
				this.testClass, this.testId);

		int start = Integer.MAX_VALUE;
		int end = 0;
		for (Interval interval : this.intervals) {
			start = Math.min(start, interval.start);
			end = Math.max(end, interval.end);
		}

		for (FSMRecognizer recognizer : this.recognizers) {
			if (recognizer.test(this.intervals, start, end)) {
				rr.recognized.put(recognizer.key(), Boolean.valueOf(true));
			}
		}
		return rr;
	}
}