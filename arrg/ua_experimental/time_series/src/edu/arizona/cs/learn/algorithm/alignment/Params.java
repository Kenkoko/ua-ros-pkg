package edu.arizona.cs.learn.algorithm.alignment;

import java.util.List;

import edu.arizona.cs.learn.timeseries.model.symbols.Symbol;

public class Params {
	public List<Symbol> seq1;
	public int min1;
	public double bonus1;
	public double penalty1;

	public List<Symbol> seq2;
	public int min2;
	public double bonus2;
	public double penalty2;

	public Normalize normalize = Normalize.regular;
	public Similarity similarity = Similarity.strings;

	public double subPenalty = -10000.0D;
	

	public Params() {
	}

	public Params(List<Symbol> seq1, List<Symbol> seq2) {
		this.seq1 = seq1;
		this.seq2 = seq2;
	}

	public void setMin(int min1, int min2) {
		this.min1 = min1;
		this.min2 = min2;
	}

	public void setBonus(double bonus1, double bonus2) {
		this.bonus1 = bonus1;
		this.bonus2 = bonus2;
	}

	public void setPenalty(double penalty1, double penalty2) {
		this.penalty1 = penalty1;
		this.penalty2 = penalty2;
	}
}