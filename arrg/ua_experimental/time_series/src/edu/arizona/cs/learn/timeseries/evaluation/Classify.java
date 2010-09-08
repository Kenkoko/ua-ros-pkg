package edu.arizona.cs.learn.timeseries.evaluation;

import java.util.ArrayList;
import java.util.List;

import edu.arizona.cs.learn.timeseries.evaluation.classifier.CAVEClassifier;
import edu.arizona.cs.learn.timeseries.evaluation.classifier.CAVEPruneClassifier;
import edu.arizona.cs.learn.timeseries.evaluation.classifier.CAVESortClassifier;
import edu.arizona.cs.learn.timeseries.evaluation.classifier.Classifier;
import edu.arizona.cs.learn.timeseries.evaluation.classifier.NearestNeighbor;

public enum Classify {
	knn {
		@Override
		public Classifier getClassifier(int k, int percent) {
			return new NearestNeighbor(k,true);
		}
		 
	},
	cave {
		@Override
		public Classifier getClassifier(int k, int percent) {
			return new CAVEClassifier(percent);
		}
	}, 
	prune {
		@Override
		public Classifier getClassifier(int k, int percent) {
			return new CAVEPruneClassifier(percent);
		}
	},
	single {
		@Override
		public Classifier getClassifier(int k, int percent) {
			return new CAVESortClassifier("single");
		}
	}, 
	complete {
		@Override
		public Classifier getClassifier(int k, int percent) {
			return new CAVESortClassifier("complete");
		}
	}, 
	average {
		@Override
		public Classifier getClassifier(int k, int percent) {
			return new CAVESortClassifier("average");
		}
	};


	/**
	 * Each enum should return the classifier associated with it.
	 * The args argument allows us to pass in arguments that are
	 * specific to a particular type of classifier
	 * @param k TODO
	 * @param percent TODO
	 * @return
	 */
	public abstract Classifier getClassifier(int k, int percent);
	
	public static List<Classify> get(String option) { 
		List<Classify> list = new ArrayList<Classify>();
		if ("all".equals(option)) { 
			list.add(knn);
			list.add(cave);
		} else if ("agglomerative".equals(option)) { 
			list.add(single);
			list.add(complete);
			list.add(average);
		} else {
			list.add(Classify.valueOf(option));
		}
		return list;
	}
}
