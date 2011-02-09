package edu.arizona.cs.learn.algorithm.alignment;

import java.util.List;

import edu.arizona.cs.learn.timeseries.model.symbols.Symbol;

public class LCS {

	/**
	 * Longest-common-subsequence.  
	 * @param seq1
	 * @param seq2
	 * @return
	 */
	public static double lcs(List<Symbol> seq1, List<Symbol> seq2) {
		int m = seq1.size();
		int n = seq2.size();

		double[] nextRow = new double[m + 1];
		double[] lastRow = new double[m + 1];
		lastRow[0] = 0.0D;
		for (int i = 1; i <= m; i++) {
			double previous = lastRow[(i - 1)];
			lastRow[i] = previous;
		}

		double[] starterCol = new double[n + 1];
		starterCol[0] = 0.0D;
		for (int i = 1; i <= n; i++) {
			double previous = starterCol[(i - 1)];
			starterCol[i] = previous;
		}

		for (int i = 1; i <= n; i++) {
			Symbol item2 = seq2.get(i - 1);
			nextRow[0] = starterCol[i];

			for (int j = 1; j <= m; j++) {
				Symbol item1 = seq1.get(j - 1);

				double choice1 = lastRow[(j - 1)] - 1000.0D;
				if (item1.equals(item2)) {
					choice1 = lastRow[(j - 1)] + 1.0D;
				}

				double choice2 = lastRow[j];
				double choice3 = nextRow[(j - 1)];
				nextRow[j] = Math.max(choice1, Math.max(choice2, choice3));
			}

			double[] tmp = lastRow;
			lastRow = nextRow;
			nextRow = tmp;
		}
		double score = lastRow[m];
		return 1.0D - score / Math.min(m, n);
	}
}
