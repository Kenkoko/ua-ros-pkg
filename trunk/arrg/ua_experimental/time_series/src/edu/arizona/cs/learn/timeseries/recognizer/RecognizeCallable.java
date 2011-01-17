package edu.arizona.cs.learn.timeseries.recognizer;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.Callable;

import edu.arizona.cs.learn.algorithm.markov.FSMRecognizer;
import edu.arizona.cs.learn.timeseries.model.Instance;
import edu.arizona.cs.learn.timeseries.model.Interval;

public class RecognizeCallable implements Callable<RecognizeCallable> {
	private List<FSMRecognizer> _recognizers;

	private int _testId;
	private String _testName;
	
	private List<Interval> _testIntervals;
	private int _start;
	private int _end;
	
	private Map<String, Boolean> _recognized;	
	private Map<String, List<Double>> _depths;
	private Map<String, List<Double>> _depthRatios;
	
	public RecognizeCallable(Map<String,FSMRecognizer> recognizers, Instance test) {
		this(new ArrayList<FSMRecognizer>(recognizers.values()), test.id(), test.name(), test.intervals());
	}
	
	public RecognizeCallable(List<FSMRecognizer> recognizers, int testId, String testName, List<Interval> test) {
		_recognizers = recognizers;
		_testId = testId;
		_testName = testName;
		_testIntervals = test;
		
		_recognized = new HashMap<String,Boolean>();
	}
	

	public RecognizeCallable call() throws Exception {
		_start = Integer.MAX_VALUE;
		_end = 0;
		for (Interval interval : _testIntervals) {
			_start = Math.min(_start, interval.start);
			_end = Math.max(_end, interval.end);
		}

		_depths = new HashMap<String,List<Double>>();
		_depthRatios = new HashMap<String,List<Double>>();

		for (FSMRecognizer recognizer : _recognizers) {
			List<Double> depths = new ArrayList<Double>();
			List<Double> ratios = new ArrayList<Double>();
			boolean recognized = recognizer.test(_testIntervals, _start, _end, depths, ratios);
			
			_recognized.put(recognizer.key(), recognized);
			_depths.put(recognizer.key(), depths);
			_depthRatios.put(recognizer.key(), ratios);
		}
		return this;
	}
	
	public String name() { 
		return _testName;
	}
	
	public int id() { 
		return _testId;
	}
	
	public int start() { 
		return _start;
	}
	
	public boolean recognized(String name) { 
		return _recognized.get(name);
	}
	
	public List<Double> depths(String name) { 
		return _depths.get(name);
	}
	
	public List<Double> depthRatios(String name) { 
		return _depthRatios.get(name);
	}
	
	
}