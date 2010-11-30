package edu.arizona.cs.learn.timeseries.evaluation;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Map;
import java.util.Random;
import java.util.concurrent.Executors;

import edu.arizona.cs.learn.algorithm.alignment.model.Instance;
import edu.arizona.cs.learn.timeseries.classification.Classifier;
import edu.arizona.cs.learn.util.Utils;

/**
 * Partitions data into training sets and testing
 * sets based on the percentage given.  
 * @author wkerr
 *
 */
public class SplitAndTest extends ClassificationTest {

	private int _repeats;
	private double _pct;
	
	private List<Instance> _training;
	private List<Instance> _testing;
	
	public SplitAndTest(int repeats, double pct) { 
		_repeats = repeats;
		_pct = pct;
	}
	
	/**
	 * Partition the given data into training and test sets 
	 * @param seed
	 * @param data
	 * @return
	 */
	public void partition(Random r, Map<String,List<Instance>> data) {
		_training = new ArrayList<Instance>();
		_testing = new ArrayList<Instance>();

		for (String className : data.keySet()) { 
			List<Instance> episodes = data.get(className);
			Collections.shuffle(episodes, r);

			int split = (int) Math.floor(_pct * (double) episodes.size());
			for (int i = 0; i < split; ++i) 
				_training.add(episodes.get(i));
			
			for (int i = split; i < episodes.size(); ++i) 
				_testing.add(episodes.get(i));
		}
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
		List<BatchStatistics> stats = new ArrayList<BatchStatistics>();
		
		Random r = new Random(seed);
		for (int i = 0; i < _repeats; ++i) { 
			partition(r, data);
			stats.add(runBatch(i, c, classNames, _training, _testing));
		}
		_execute.shutdown();
		_execute = null;
		
		return stats;
	}
}
