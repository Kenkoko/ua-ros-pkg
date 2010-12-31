package edu.arizona.cs.learn.timeseries.classification;

import java.util.ArrayList;
import java.util.List;

public enum Classify {
	knn {
		public Classifier getClassifier(ClassifyParams params) {
			return new NearestNeighbor(params);
		}
	},
	cave {
		public Classifier getClassifier(ClassifyParams params) {
			return new CAVEClassifier(params);
		}
	},
	single {
		public Classifier getClassifier(ClassifyParams params) {
			params.method = "single";
			return new CAVEClassifier(params);
		}
	},
	complete {
		public Classifier getClassifier(ClassifyParams params) {
			params.method = "complete";
			return new CAVEClassifier(params);
		}
	},
	average {
		public Classifier getClassifier(ClassifyParams params) {
			params.method = "average";
			return new CAVEClassifier(params);
		}
	},
	prune { 
		public Classifier getClassifier(ClassifyParams params) {
			return new CAVEClassifier(params);
		}
	};

	public abstract Classifier getClassifier(ClassifyParams params);

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