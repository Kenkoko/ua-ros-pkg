package edu.arizona.cs.learn.timeseries.classification;

import java.util.List;

import edu.arizona.cs.learn.algorithm.alignment.model.Instance;
import edu.arizona.cs.learn.timeseries.model.Episode;
import edu.arizona.cs.learn.util.SequenceType;

public abstract class Classifier {
	protected SequenceType _type;

	public Classifier(SequenceType type) {
		_type = type;
	}

	public abstract String getName();

	public abstract void train(int fold, List<Instance> training);
	public abstract void train(List<Instance> training);

	public abstract void trainEpisodes(int x, List<Episode> training, SequenceType type, boolean shuffle);
	public abstract void trainEpisodes(List<Episode> training, SequenceType type, boolean shuffle);

	public abstract String test(Instance testInstance);
}