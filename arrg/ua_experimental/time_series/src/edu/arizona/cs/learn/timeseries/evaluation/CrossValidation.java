package edu.arizona.cs.learn.timeseries.evaluation;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Random;
import java.util.TreeMap;
import java.util.concurrent.Executors;

import edu.arizona.cs.learn.timeseries.classification.Classifier;
import edu.arizona.cs.learn.timeseries.model.Instance;
import edu.arizona.cs.learn.util.Utils;

public class CrossValidation extends ClassificationTest {
	
	private int _nFolds;
	
	public CrossValidation(int nFolds) { 
		_nFolds = nFolds;
	}

	/**
	 * Partition the given data into k sets for cross validation
	 * @param seed
	 * @param data
	 * @return
	 */
	public List<Map<String,List<Instance>>> partition(long seed, Map<String,List<Instance>> data) {
		Random r = new Random(seed);

		List<Map<String,List<Instance>>> sets = new ArrayList<Map<String,List<Instance>>>();
		for (int i = 0; i < _nFolds; i++) {
			Map<String,List<Instance>> map = new TreeMap<String,List<Instance>>();
			for (String c : data.keySet()) 
				map.put(c, new ArrayList<Instance>());
			sets.add(map);
		}

		for (String className : data.keySet()) { 
			List<Instance> episodes = data.get(className);
			Collections.shuffle(episodes, r);
			
			for (int i = 0; i < episodes.size(); ++i) {
				sets.get(i % _nFolds).get(className).add(episodes.get(i));
			}
		}
		return sets;
	}
	
	
	/**
	 * run CrossValidation on the data and automatically partition
	 * the data into training and test sets.
	 * @param seed - so that you can control the splitting of k
	 * @params classNames - so that the classNames order is unique
	 * @param data
	 * @param c
	 * @return
	 */
	public List<BatchStatistics> run(long seed, List<String> classNames, Map<String,List<Instance>> data, Classifier c) { 
		_execute = Executors.newFixedThreadPool(Utils.numThreads);
		List<BatchStatistics> foldStats = new ArrayList<BatchStatistics>();
		
		List<Map<String,List<Instance>>> sets = partition(seed, data);
		for (int i = 0; i < _nFolds; ++i) { 
			Map<String,List<Instance>> train = new HashMap<String,List<Instance>>();
			for (int j = 0; j < _nFolds; ++j) { 
				if (i == j)
					continue;
				
				Map<String,List<Instance>> map = sets.get(j);
				for (String key : map.keySet()) { 
					List<Instance> trainList = train.get(key);
					if (trainList == null) { 
						trainList = new ArrayList<Instance>();
						train.put(key, trainList);
					}
					trainList.addAll(map.get(key));
				}
			}

			Map<String,List<Instance>> test = sets.get(i);
			foldStats.add(runBatch(i, c, classNames, train, test));

		}
		_execute.shutdown();
		return foldStats;
	}
}
