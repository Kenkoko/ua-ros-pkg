package edu.arizona.cs.learn.timeseries.classification;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;

import org.apache.log4j.Logger;

import edu.arizona.cs.learn.algorithm.alignment.Params;
import edu.arizona.cs.learn.algorithm.alignment.SequenceAlignment;
import edu.arizona.cs.learn.algorithm.alignment.model.Instance;
import edu.arizona.cs.learn.timeseries.model.Episode;
import edu.arizona.cs.learn.timeseries.model.Score;
import edu.arizona.cs.learn.timeseries.model.Signature;
import edu.arizona.cs.learn.timeseries.model.SignatureCallable;
import edu.arizona.cs.learn.util.SequenceType;
import edu.arizona.cs.learn.util.Utils;

public class CAVEClassifier extends Classifier {
	private static Logger logger = Logger.getLogger(CAVEClassifier.class);
	protected int _percent;
	protected double _pct;
	protected boolean _prune;
	protected boolean _load;
	
	protected String _method;

	protected Map<String, Signature> _map;
	protected int _folds;

	public CAVEClassifier(SequenceType type, int percent, boolean prune, boolean fromFile, int folds) {
		super(type);

		this._folds = folds;
		this._percent = percent;
		this._pct = (this._percent / 100.0D);

		this._load = fromFile;
		this._prune = prune;
	}

	public CAVEClassifier(SequenceType type, int percent, boolean prune, boolean fromFile, int folds, String method) {
		super(type);

		this._folds = folds;
		this._percent = percent;
		this._pct = (this._percent / 100.0D);

		this._load = fromFile;
		this._prune = prune;

		this._method = method;
	}

	public String getName() {
		if (this._method != null) {
			return "cave-" + this._method;
		}
		return "cave-" + this._percent;
	}

	public String test(Instance instance) {
		Params params = new Params();
		params.setMin(5, 0);
		params.setBonus(1.0D, 0.0D);
		params.setPenalty(-1.0D, 0.0D);

		List<Score> scores = new ArrayList<Score>();
		for (Signature signature : this._map.values()) {
			params.seq1 = signature.signature();
			params.min1 = (int) Math.round(signature.trainingSize() * _pct);
			params.seq2 = instance.sequence();

			Score score = new Score();
			score.key = signature.key();
			score.distance = SequenceAlignment.distance(params);
			scores.add(score);
		}

		Collections.sort(scores, new Comparator<Score>() {
			public int compare(Score o1, Score o2) {
				return Double.compare(o1.distance, o2.distance);
			}
		});
		return scores.get(0).key;
	}

	public Map<String,Long> train(int x, Map<String,List<Instance>> training) {
		_map = new HashMap<String,Signature>();

		if (_method != null) {
			return doTrain(training);
		} else if (this._load) {
			return load(x, training.keySet());
		} else {
			return learn(x, training);
		}
	}
	
	public void trainEpisodes(int x, List<Episode> training, SequenceType type, boolean shuffle) { 
		_map = new HashMap<String,Signature>();

		if (_method != null) 
			throw new RuntimeException("Not yet implemented!!!");
		else if (_load) {
			Set<String> classNames = new HashSet<String>();
			for (Episode episode : training) 
				classNames.add(episode.name());
			load(x, classNames);
		} else {
			throw new RuntimeException("Not yet implemented!!!!");
		}
	}

	/**
	 * Do the training which involves building the signatures for
	 * each of the different class names (agglomeratively).
	 * @param training
	 * @return the timing information (how long it took by class)
	 */
	private Map<String,Long> doTrain(Map<String,List<Instance>> training) {
		Map<String,Long> timing = new HashMap<String,Long>();
		for (String key : training.keySet()) {
			long start = System.currentTimeMillis();
			Signature s = Signature.agglomerativeTraining(_method, training.get(key));
			long end = System.currentTimeMillis();
			
			timing.put(key, end-start);
			_map.put(key, s);
		}
		return timing;
	}

	/**
	 * Load in the data from file.... should be much quicker than training
	 * @param x
	 * @param keys
	 */
	private Map<String,Long> load(int x, Collection<String> keys) {
		String dir = "data/cross-validation/k" + _folds + "/fold-" + x
				+ "/" + this._type + "/";
		String suffix = ".xml";
		if (_prune) {
			suffix = "-prune.xml";
		}

		Map<String,Long> timing = new HashMap<String,Long>();
		for (String key : keys) {
			long start = System.currentTimeMillis();
			Signature s = Signature.fromXML(dir + key + suffix);
			long end = System.currentTimeMillis();

			timing.put(key, end-start);
			_map.put(key, s);
		}
		return timing;
	}

	/**
	 * learn the signatures before classification... This method
	 * is multithreaded to help speed things up.
	 * @param x
	 * @param training
	 */
	private Map<String,Long> learn(int x, Map<String,List<Instance>> training) {
		ExecutorService execute = Executors.newFixedThreadPool(Utils.numThreads);

		List<Future<SignatureCallable>> futureList = new ArrayList<Future<SignatureCallable>>();
		for (String key : training.keySet()) 
			futureList.add(execute.submit(new SignatureCallable(key, _prune, 3, training.get(key))));
			
		Map<String,Long> timing = new HashMap<String,Long>();
		for (Future<SignatureCallable> results : futureList) {
			try {
				SignatureCallable sc = results.get();
				Signature signature = sc.signature();

				timing.put(signature.key(), sc.duration());
				_map.put(signature.key(), signature);
			} catch (Exception e) { 
				e.printStackTrace();
			}
		}
		execute.shutdown();
		return timing;
	}
	
	public Map<String,Long> train(Map<String,List<Instance>> training) {
		throw new RuntimeException("Not yet implemented!!");
	}
	
	public void trainEpisodes(List<Episode> training, SequenceType type, boolean shuffle) { 
		throw new RuntimeException("Not yet implemented!!!");
	}
}