package edu.arizona.cs.learn.timeseries.classification;

import java.util.ArrayList;
import java.util.List;

import edu.arizona.cs.learn.util.SequenceType;

public enum Classify {
	knn {
		public Classifier getClassifier(SequenceType type,
				int k, int percent, boolean fromFile, int folds) {
			return new NearestNeighbor(type, k, true);
		}
	},
	cave {
		public Classifier getClassifier(SequenceType type,
				int k, int percent, boolean fromFile, int folds) {
			return new CAVEClassifier(type, percent, false, fromFile, folds);
		}
	},
	single {
		public Classifier getClassifier(SequenceType type,
				int k, int percent, boolean fromFile, int folds) {
			return new CAVEClassifier(type, percent, true, fromFile, folds, "single");
		}
	},
	complete {
		public Classifier getClassifier(SequenceType type,
				int k, int percent, boolean fromFile, int folds) {
			return new CAVEClassifier(type, percent, true, fromFile, folds, "complete");
		}
	},
	average {
		public Classifier getClassifier(SequenceType type,
				int k, int percent, boolean fromFile, int folds) {
			return new CAVEClassifier(type, percent, true, fromFile, folds, "average");
		}
	},
	prune { 
		public Classifier getClassifier(SequenceType type,
				int k, int percent, boolean fromFile, int folds) {
			return new CAVEClassifier(type, percent, true, fromFile, folds);
		}
	};

	public abstract Classifier getClassifier(SequenceType sequenceType, int k, int percent, boolean fromFiles, int folds);

	public static List<Classify> get(String option) {
		List<Classify> list = new ArrayList<Classify>();
		if ("all".equals(option)) {
			list.add(knn);
			list.add(cave);
		} else if ("agg".equals(option)) {
			list.add(single);
			list.add(complete);
			list.add(average);
		} else {
			list.add(valueOf(option));
		}
		return list;
	}
}