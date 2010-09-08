package edu.arizona.cs.learn.timeseries.evaluation.classifier;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.apache.log4j.Logger;

import edu.arizona.cs.learn.algorithm.alignment.Params;
import edu.arizona.cs.learn.algorithm.alignment.SequenceAlignment;
import edu.arizona.cs.learn.algorithm.alignment.model.Instance;
import edu.arizona.cs.learn.timeseries.evaluation.FoldStatistics;
import edu.arizona.cs.learn.timeseries.model.Signature;

public class NearestNeighbor implements Classifier {
    private static Logger logger = Logger.getLogger(NearestNeighbor.class);

	private int _k;
	private boolean _weighted;

	private List<Instance> _trainingData;
	
	public NearestNeighbor(int k, boolean weighted) { 
		_k = k;
		_weighted = weighted;
	}
	
	public String getName() { 
		return "knn";
	}
	
	public void addData(FoldStatistics fold) { 
		// do nothing.
	}

	public String test(Instance instance) {
		Params params = new Params();
		params.setMin(0, 0);
		params.setBonus(1,1);
		params.setPenalty(0, 0);
		
		
//		logger.debug("\t\tTesting: " + instance.name() + " training size: " + _trainingData.size());
		List<Distance> distanceList = new ArrayList<Distance>();
		for (Instance tmp : _trainingData) { 
			params.seq1 = instance.sequence();
			params.seq2 = tmp.sequence();
			double distance = SequenceAlignment.distance(params);
//			double distance = SequenceAlignment.lcs(params.seq1, params.seq2);
			assert(distance <= 1.0);
			
//			logger.debug("\t\t\tDistance: " + distance);
			distanceList.add(new Distance(tmp, distance));
		}
//		logger.debug("\t\tDistances Found: " + instance.name());
		
//		logger.debug("all: " + all.size());
		Collections.sort(distanceList, new Comparator<Distance>() {
			public int compare(Distance o1, Distance o2) {
				return Double.compare(o1.d, o2.d);
			} 
		});
		
		// this classifier returns the most probable class (or the class
		// seen the most)
		String maxClass = null;
		double max = 0;
		
		Map<String,Double> map = new HashMap<String,Double>();
		for (int i = 0; i < distanceList.size() && i < _k; ++i) { 
			Distance distance = distanceList.get(i);

			double inc = 1;
			if (_weighted)  
				inc = 1 / distance.d;
//			logger.debug(i.getKey());
			if (map.containsKey(distance.instance.name())) { 
				map.put(distance.instance.name(), map.get(distance.instance.name()) + inc);
			} else { 
				map.put(distance.instance.name(), inc);
			}
			
			if (map.get(distance.instance.name()) > max) { 
				max = map.get(distance.instance.name());
				maxClass = distance.instance.name();
			}
		}
		
//		if (!maxClass.equals(instance.name())) { 
//			logger.debug("ERROR - " + instance.name() + " classified as " + maxClass);
//			for (Instance i : nn) { 
//				logger.debug("\t" + i.id() + " " + i.name() + " " + i.get("d"));
//			}
//		}
//		logger.debug("class: " + maxClass + " - " + max);
		return maxClass;
	}

	public void train(List<List<Instance>> data, int x) {
		_trainingData = new ArrayList<Instance>();
		for (int i = 0; i < data.size(); ++i) { 
			if (i == x) 
				continue;
			
			for (Instance instance : data.get(i)) { 
				_trainingData.add(instance);
			}
		}
	}
	
	public void train(List<Instance> data) { 
		_trainingData = new ArrayList<Instance>(data);
	}
	
	public void printError(Instance instance, int id) { 
		
	}
}

class Distance { 
	public Instance instance;
	public double d;
	
	public Distance(Instance instance, double d) { 
		this.instance = instance;
		this.d = d;
	}
}
