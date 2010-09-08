package edu.arizona.cs.learn.timeseries.classification;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

import org.apache.log4j.Logger;

import edu.arizona.cs.learn.algorithm.alignment.Params;
import edu.arizona.cs.learn.algorithm.alignment.SequenceAlignment;
import edu.arizona.cs.learn.algorithm.alignment.model.Instance;
import edu.arizona.cs.learn.timeseries.distance.Distances;
import edu.arizona.cs.learn.timeseries.evaluation.FoldStatistics;
import edu.arizona.cs.learn.timeseries.evaluation.cluster.Clustering;
import edu.arizona.cs.learn.timeseries.model.AgglomerativeNode;
import edu.arizona.cs.learn.timeseries.model.Episode;
import edu.arizona.cs.learn.timeseries.model.Score;
import edu.arizona.cs.learn.timeseries.model.Signature;
import edu.arizona.cs.learn.util.SequenceType;

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

	public void addData(FoldStatistics fold) {
		for (Signature signature : this._map.values()) {
			int minSeen = (int) Math
					.round(signature.trainingSize() * this._pct);
			fold.signatureSize.addValue(SequenceAlignment.subset(
					signature.signature(), minSeen).size());
		}
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

	public void train(int x, List<Instance> training) {
		_map = new HashMap<String,Signature>();

		if (_method != null) {
			doTrain(training);
		} else if (this._load) {
			// Determine the different classes available
			Set<String> classNames = new HashSet<String>();
			for (Instance instance : training) 
				classNames.add(instance.name());
			load(x, classNames);
		} else {
			learn(x, training);
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

	private void doTrain(List<Instance> training) {
		Map<String,List<Instance>> instances = new HashMap<String,List<Instance>>();
		for (Instance instance : training) {
			List<Instance> list = instances.get(instance.name());
			if (list == null) {
				list = new ArrayList<Instance>();
				instances.put(instance.name(), list);
			}

			list.add(instance);
		}

		for (String key : instances.keySet()) {
			Signature s = Signature.agglomerativeTraining(_method, instances.get(key));
			_map.put(key, s);
		}
	}

	private void load(int x, Set<String> classSet) {
		String dir = "data/cross-validation/k" + this._folds + "/fold-" + x
				+ "/" + this._type + "/";
		String suffix = ".xml";
		if (_prune) {
			suffix = "-prune.xml";
		}

		for (String key : classSet) {
			logger.debug("loading: " + dir + key + suffix);
			Signature s = Signature.fromXML(dir + key + suffix);
			_map.put(key, s);
		}
	}

	private void learn(int x, List<Instance> training) {
		Map<String,List<Instance>> instances = new HashMap<String,List<Instance>>();
		for (Instance instance : training) {
			List<Instance> list = instances.get(instance.name());
			if (list == null) {
				list = new ArrayList<Instance>();
				instances.put(instance.name(), list);
			}

			list.add(instance);
		}

		for (String key : instances.keySet()) {
			Signature s = new Signature(key);
			List<Instance> list = instances.get(key);
			for (int i = 1; i <= list.size(); i++) {
				s.update(list.get(i-1).sequence());
				if ((_prune) && (i % 10 == 0)) {
					s = s.prune(3);
				}
			}
			
			// Fifty percent pruning....
			s = s.prune(s.trainingSize()/2);
			_map.put(key, s);
		}
	}
	
	public void train(List<Instance> training) {
		throw new RuntimeException("Not yet implemented!!");
	}
	
	public void trainEpisodes(List<Episode> training, SequenceType type, boolean shuffle) { 
		throw new RuntimeException("Not yet implemented!!!");
	}
}