package edu.arizona.cs.learn.algorithm.alignment;

import java.util.List;

import edu.arizona.cs.learn.timeseries.model.symbols.Symbol;

public enum Normalize {
	knn {
		@Override
		public double normalize(Params params, List<Symbol> seq1,
				List<Symbol> seq2, double score) {
			double min = Math.min(seq1.size(), seq2.size());
			double max = Math.max(seq1.size(), seq2.size());
			double sum = seq1.size() + seq2.size();
			double maxScore = sum - (max - min);
			return 1.0D - score / maxScore;
		}
	}, 
	signature {
		@Override
		public double normalize(Params params, List<Symbol> seq1,
				List<Symbol> seq2, double score) {
			throw new RuntimeException("Not yet implemented!");
		}
	}, 
	regular {
		@Override
		public double normalize(Params params, List<Symbol> seq1,
				List<Symbol> seq2, double score) {
			double worstScore = 0.0D;
			double bestScore = 0.0D;
			for (Symbol obj : seq1) {
				bestScore += params.bonus1 * obj.weight();
				worstScore += params.penalty1 * obj.weight();
			}

			for (Symbol obj : seq2) {
				bestScore += params.bonus2 * obj.weight();
				worstScore += params.penalty2 * obj.weight();
			}
			worstScore = Math.abs(worstScore);
			return 1.0D - (score + worstScore) / (bestScore + worstScore);
		}
	}, 
	none {
		@Override
		public double normalize(Params params, List<Symbol> seq1,
				List<Symbol> seq2, double score) {
			return score;
		}
	};

	/**
	 * Given some information about the sequence alignment
	 * we normalize the score so that it is between 0 and
	 * 1.  0 = perfect match and 1 = perfect mismatch
	 * @param params
	 * @param seq1
	 * @param seq2
	 * @param score
	 * @return
	 */
	public abstract double normalize(Params params, 
			List<Symbol> seq1, List<Symbol> seq2,  double score);
}
