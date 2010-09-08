package edu.arizona.cs.learn.timeseries.recognizer;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.apache.log4j.Logger;

public class RecognizerStatistics {
	private static Logger logger = Logger.getLogger(RecognizerStatistics.class);
	public String key;
	
	public List<Integer> truePositives;
	public List<Integer> falseNegative;
	
	private Map<String, List<Integer>> _falsePositiveMap;
	private Map<String, List<Integer>> _trueNegativeMap;

	public RecognizerStatistics(String key) {
		this.key = key;

		this.truePositives = new ArrayList<Integer>();
		this.falseNegative = new ArrayList<Integer>();

		this._falsePositiveMap = new HashMap<String,List<Integer>>();
		this._trueNegativeMap = new HashMap<String,List<Integer>>();
	}

	public void falsePositive(String className, int id) {
		List<Integer> falsePositive = _falsePositiveMap.get(className);
		if (falsePositive == null) {
			falsePositive = new ArrayList<Integer>();
			_falsePositiveMap.put(className, falsePositive);
		}

		falsePositive.add(Integer.valueOf(id));
	}

	public void trueNegative(String className, int id) {
		List<Integer> trueNegative = _trueNegativeMap.get(className);
		if (trueNegative == null) {
			trueNegative = new ArrayList<Integer>();
			_trueNegativeMap.put(className, trueNegative);
		}

		trueNegative.add(Integer.valueOf(id));
	}

	public double tp() {
		return this.truePositives.size();
	}

	public double fn() {
		return this.falseNegative.size();
	}

	public double fp() {
		double fp = 0.0D;
		for (List<Integer> list : _falsePositiveMap.values())
			fp += list.size();
		return fp;
	}

	public double tn() {
		double tn = 0.0D;
		for (List<Integer> list : _trueNegativeMap.values())
			tn += list.size();
		return tn;
	}

	public double precision() {
		double tp = this.truePositives.size();
		double fp = 0.0D;
		for (List<Integer> list : _falsePositiveMap.values())
			fp += list.size();
		return tp / (tp + fp);
	}

	public double recall() {
		double tp = this.truePositives.size();
		double fn = this.falseNegative.size();
		return tp / (tp + fn);
	}

	public double fscore() {
		double p = precision();
		double r = recall();

		return 2.0D * (p * r / (p + r));
	}
}