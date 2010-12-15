package edu.arizona.cs.learn.timeseries.evaluation;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;


public class BatchStatistics {
	private String _cName;
	private List<String> _classNames;
	
	private List<String> _actualClass;
	private List<String> _predictedClass;
	private List<Boolean> _detail;
	private List<Long> _duration;
	
	private Map<String,Integer> _trainingCount;
	private Map<String,Long>    _trainingTime;
	private Map<String,Double>  _avgTrainingSize;
	
	private int[][] _confMatrix;
	
	public BatchStatistics(String classifierName, List<String> classNames) { 
		_cName = classifierName;
		_classNames = classNames;
		
		_confMatrix = new int[classNames.size()][classNames.size()];
		_detail = new ArrayList<Boolean>();
		_actualClass = new ArrayList<String>();
		_predictedClass = new ArrayList<String>();
		_duration = new ArrayList<Long>();
		
		_trainingCount = new HashMap<String,Integer>();
		_trainingTime = new HashMap<String,Long>();
		_avgTrainingSize = new HashMap<String,Double>();
	}
	
	public double accuracy() { 
		double right = 0;
		for (Boolean b : _detail) { 
			if (b) ++right;
		}
		
		return right / (double) _detail.size();
	}
	
	public String cName() { 
		return _cName;
	}

	public int[][] confMatrix() { 
		return _confMatrix;
	}	
	
	public double[][] normalizeConfMatrix() { 
		double[][] confMatrix = new double[_classNames.size()][_classNames.size()];
		for (int i = 0; i < _classNames.size(); ++i) { 
			double totals = 0;
			for (int j = 0; j < _classNames.size(); ++j)  
				totals += _confMatrix[i][j];

			for (int j = 0; j < _classNames.size(); ++j) 
				confMatrix[i][j] = (double) _confMatrix[i][j] / totals;
		}
		return confMatrix;
	}
	
	public void addTestDetail(String actual, String predicted, Long duration) { 
		_actualClass.add(actual);
		_predictedClass.add(predicted);
		
		if (actual.equals(predicted)) {
			_detail.add(true);
		} else { 
			_detail.add(false);
		}
		
		int correctIndex = _classNames.indexOf(actual);
		int classifyIndex = _classNames.indexOf(predicted);
		_confMatrix[correctIndex][classifyIndex] += 1;

		_duration.add(duration);
	}
	
	public void addTrainingDetail(String className, int count, Long duration, Double avgSize) { 
		_trainingCount.put(className, count);
		_trainingTime.put(className, duration);
		_avgTrainingSize.put(className, avgSize);
	}
	
	public void addTrainingDetail(String className, int count, Long duration) { 
		addTrainingDetail(className, count, duration, 0.0);
	}
	
	public String toCSV(String prefix, String suffix) { 
		StringBuffer buf = new StringBuffer();
		for (int i = 0; i < _detail.size(); ++i) { 
			buf.append(prefix);

			// Add in all of the details.
			buf.append(i + "," + _actualClass.get(i) + "," + _predictedClass.get(i) + ",");
			buf.append((_detail.get(i) ? 1 : 0) + ",");
			buf.append(_trainingTime.get(_actualClass.get(i)) + "," + _trainingCount.get(_actualClass.get(i)) + ",");
			buf.append(_avgTrainingSize.get(_actualClass.get(i)) + ",");
			buf.append(_duration.get(i));
			
			buf.append(suffix + "\n");
		}
		return buf.toString();
	}
	
	public static String csvHeader() { 
		return "test_index,actual_class,predicted_class,correct,training_time,training_count,avgTrainingSize,testing_time";
	}
}
