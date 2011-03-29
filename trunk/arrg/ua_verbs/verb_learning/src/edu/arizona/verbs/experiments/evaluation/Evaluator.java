package edu.arizona.verbs.experiments.evaluation;

import java.util.Arrays;

public class Evaluator {

	public static EvaluationResults evaluateAllLocations(boolean[] actual) {
		boolean[] allCutPoints = new boolean[actual.length];
		Arrays.fill(allCutPoints, true);
		return evaluate(allCutPoints, actual);
	}

	public static EvaluationResults evaluate(boolean[] algorithm, boolean[] actual) {
		int numTruePositives = 0;
		int numFalsePositives = 0;
		int numTrueNegatives = 0;
		int numFalseNegatives = 0;

		int numNegatives = 0;

		if (algorithm.length != actual.length) {
			throw new RuntimeException("ARRAY LENGTHS DO NOT MATCH: " + algorithm.length + " should equal " + actual.length);
		}

		numTruePositives = 0;
		numFalsePositives = 0;
		numTrueNegatives = 0;
		numFalseNegatives = 0;
		for (int i = 0; i < actual.length; ++i) {
			if (algorithm[i]) {
				if (actual[i]) {
					numTruePositives++;
				} else {
					numFalsePositives++;
					numNegatives++;
				}
			} else {
				if (actual[i]) {
					numFalseNegatives++;
				} else {
					numTrueNegatives++;
					numNegatives++;
				}
			}
		}

		int total = numTruePositives + numFalsePositives + numTrueNegatives
				+ numFalseNegatives;

		if (total != algorithm.length) {
			throw new RuntimeException("YO WE GOT PROBLEMS! " + total + " "
					+ algorithm.length);
		}

		EvaluationResults results = new EvaluationResults();
		results.precision = numTruePositives
				/ ((double) numTruePositives + numFalsePositives);
		results.recall = numTruePositives
				/ ((double) numTruePositives + numFalseNegatives);

		if (Double.isNaN(results.precision)) {
			results.precision = 0.0;
		}
		
		if (Double.isNaN(results.recall)) {
			results.recall = 0.0;
		}
		
		
		return results;
	}
}
