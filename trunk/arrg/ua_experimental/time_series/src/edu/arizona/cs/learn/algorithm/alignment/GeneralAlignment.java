package edu.arizona.cs.learn.algorithm.alignment;

import java.util.ArrayList;
import java.util.List;

import org.apache.log4j.Logger;

import edu.arizona.cs.learn.timeseries.model.symbols.Symbol;

public class GeneralAlignment {
	private static Logger logger = Logger.getLogger(GeneralAlignment.class);

	
	/**
	 * Mash together to the two sequences and create one sequence that is 
	 * increments weights where the objects match and inserts new items where they
	 * don't match.
	 * @param seq1
	 * @param seq2
	 * @return
	 */
	public static List<Symbol> combineAlignments(List<Symbol> seq1, List<Symbol> seq2) {
		assert (seq1.size() == seq2.size());
		List<Symbol> results = new ArrayList<Symbol>();
		for (int i = 0; i < seq1.size(); i++) {
			Symbol item1 = seq1.get(i);
			Symbol item2 = seq2.get(i);
			
			if (item1 == null && item2 == null)
				throw new RuntimeException("Alignment cannot choose to align two null items!");

			if (item1 != null && item2 != null) 
				results.add(item1.merge(item2));
			else if (item1 != null)
				results.add(item1.copy());
			else
				results.add(item2.copy());
		}
		return results;
	}
	
	/**
	 * Select a subset of the objects in the sequences.  Specifically
	 * those that have been seen at least minSeen times.
	 * @param seq
	 * @param minSeen
	 * @return
	 */
	public static List<Symbol> subset(List<Symbol> seq, int minSeen) {
		if (minSeen == 0) 
			return seq;

		List<Symbol> seq1 = new ArrayList<Symbol>();
		double seenCount = 0.0D;
		for (Symbol obj : seq) {
			if (obj.weight() >= minSeen) {
				seq1.add(obj);
				seenCount += obj.weight();
			}
		}

		if (seq1.size() == 0) {
			return seq;
		}
		return seq1;
	}

	/**
	 * Perform alignment with the given parameters.  Wrapped like
	 * this in case we want to try out different alignment methods.
	 * @param params
	 * @return
	 */
	public static Report align(Params params) {
		return alignCheckp(params);
	}
	
	
	/**
	 * Perform the sequence alignment building the table in 
	 * main memory.
	 * @param params
	 * @return
	 */
	public static Report alignWithCons(Params params) {
		class Cons {
			public char value;
			public Cons next;
			
			public Cons() { 
				next = null;
			}
			
			public Cons(char value, Cons next) { 
				this.value = value;
				this.next = next;
			}
		}
		
		class Cell {
			public double score;
			public Cons directions;
			
			public Cell(double score) { 
				this.score = score;
				this.directions = null;
			}
		}
			
		boolean printTable = false;

		List<Symbol> seq1 = subset(params.seq1, params.min1);
		List<Symbol> seq2 = subset(params.seq2, params.min2);

		int m = seq1.size();
		int n = seq2.size();

		Cell[] nextRow = new Cell[m + 1];
		Cell[] lastRow = new Cell[m + 1];
		lastRow[0] = new Cell(0.0D);

		if (printTable)
			System.out.print("0 & ");
		for (int i = 1; i <= m; i++) {
			Symbol obj = seq1.get(i - 1);
			Cell previous = lastRow[(i - 1)];

			double value = previous.score + obj.weight() * params.penalty1;

			Cell next = new Cell(value);
			next.directions = new Cons('l', previous.directions);

			if (printTable) {
				System.out.print(value + " & ");
			}
			lastRow[i] = next;
		}

		if (printTable) {
			System.out.println();
		}

		Cell[] starterCol = new Cell[n + 1];
		starterCol[0] = new Cell(0.0D);
		for (int i = 1; i <= n; i++) {
			Symbol obj = seq2.get(i - 1);
			Cell previous = starterCol[(i - 1)];

			double value = previous.score + obj.weight() * params.penalty2;

			Cell next = new Cell(value);
			next.directions = new Cons('u', previous.directions);
			starterCol[i] = next;
		}

		for (int i = 1; i <= n; i++) {
			Symbol item2 = seq2.get(i - 1);

			nextRow[0] = starterCol[i];

			if (printTable)
				System.out.print(nextRow[0].score + " & ");
			for (int j = 1; j <= m; j++) {
				Symbol item1 = seq1.get(j - 1);

				Cell diag = lastRow[(j - 1)];
				Cell up = lastRow[j];
				Cell left = nextRow[(j - 1)];
				
				// Substitution will always be allowed and should have
				// Do not substitute if we have a complete mismatch though.
				double compare = params.similarity.similarity(item1, item2);

				// if we have at least matched one thing....then it's not
				// a complete substitution
				double choice1 = diag.score + params.subPenalty;
				if (compare > 0) { 
					double v1 = item1.weight();
					double v2 = item2.weight();
					
					choice1 = diag.score + (params.bonus1*v1*compare) + (params.bonus2*v2*compare);
				}
				
				double choice2 = up.score + (params.penalty2 * item2.weight());
				double choice3 = left.score + (params.penalty1 * item1.weight());

				Cell c = new Cell(Math.max(choice1, Math.max(choice2, choice3)));
				if ((choice1 >= choice2) && (choice1 >= choice3)) {
					c.directions = new Cons('d', diag.directions);
				} else if ((choice2 >= choice3) && (choice2 > choice1)) {
					c.directions = new Cons('u', up.directions);
				} else if ((choice3 > choice2) && (choice3 > choice1)) {
					c.directions = new Cons('l', left.directions);
				} else {
					logger.error("Error occurred [" + i + "," + j + "] "
							+ item1.toString() + " " + item2.toString());
					throw new RuntimeException("Weird: [" + i + "," + j + "] "
							+ item1.weight() + " " + item2.weight() + " "
							+ choice1 + " " + choice2 + " " + choice3);
				}
				nextRow[j] = c;

				if (printTable)
					System.out.print(c.score + " & ");
			}
			if (printTable) {
				System.out.println();
			}
			Cell[] tmp = lastRow;
			lastRow = nextRow;
			nextRow = tmp;
		}

		Cell best = lastRow[m];
		Report sc = new Report();
		sc.s1Size = m;
		sc.s2Size = n;
		sc.score = params.normalize.normalize(params, seq1, seq2, best.score);

		int i = m - 1;
		int j = n - 1;

		Cons current = best.directions;
		while (current != null) {
			switch (current.value) {
			case 'd':
				sc.add(seq1.get(i), seq2.get(j));
				i--;
				j--;
				break;
			case 'l':
				sc.add(seq1.get(i), null);
				i--;
				break;
			case 'u':
				sc.add(null, seq2.get(j));
				j--;
			}

			current = current.next;
		}

		assert ((i == -1) && (j == -1)) : ("i: " + i + " j: " + j);
		sc.finish();
		return sc;
	}	
	
	/**
	 * Code structure courtesy of David Powell
	 * http://drp.id.au/index.shtml
	 * 	Time Complexity: O(n*n)  Space Complexity: O(n) 
	 * @param params
	 * @return
	 */
	public static Report alignCheckp(Params params) { 
		List<Symbol> A = subset(params.seq1, params.min1);
		List<Symbol> B = subset(params.seq2, params.min2);
		
//		logger.debug("Sizes: " + A.size() + " " + B.size());
		Status status = new Status();
		status.alignment = new int[A.size() + B.size() + 1];
		status.alignPos = 0;

		double distance = alignCheckp(params, A, B, 0, 0, A.size(), B.size(), status);
		double worstScore = 0.0D;
		double bestScore = 0.0D;
		for (Symbol obj : params.seq1) {
			bestScore += params.bonus1 * obj.weight();
			worstScore += params.penalty1 * obj.weight();
		}

		for (Symbol obj : params.seq2) {
			bestScore += params.bonus2 * obj.weight();
			worstScore += params.penalty2 * obj.weight();
		}
		worstScore = Math.abs(worstScore);


		Report report = new Report();
		report.s1Size = A.size();
		report.s2Size = B.size();
		report.score = 1.0D - (distance + worstScore) / (bestScore + worstScore);
		
		int i = 0;
		int j = 0;
		for (int pos = 0; pos < status.alignPos; ++pos) { 
			int option = status.alignment[pos];
			switch (option) {
			case 0:
				report.add(A.get(i), B.get(j));
				++i; ++j;
				break;
			case 1:
				report.add(null, B.get(j));
				++j;
				break;
			case 2:
				report.add(A.get(i), null);
				++i;
				break;
			}
		}

		// TODO: ensure that we don't need to call 
		// report.finish() before returning.
		return report;
	}
	
	/**
	 * Do the recursive grunt work to determine the proper
	 * alignment.
	 * @param params
	 * @param A
	 * @param B
	 * @param a0
	 * @param b0
	 * @param a1
	 * @param b1
	 * @param status
	 * @return
	 */
	private static double alignCheckp(Params params, 
			List<Symbol> A, List<Symbol> B,
			int a0, int b0, int a1, int b1, Status status) { 
	
//		logger.debug("alignCheckp -- " + a0 + " " + b0 + " " + a1 + " " + b1 + " -- " + status.alignPos);
//		StringBuffer buf = new StringBuffer();
//		for (int i = 0; i < status.alignPos; ++i) { 
//			buf.append(status.alignment[i] + " ");
//		}
//		logger.debug("   current alignment: " + buf);
		
		// Test for simple cases
		if (a0 == a1) { 
			for (int i = b0; i < b1; ++i) {
				status.alignment[status.alignPos] = 1; // 1 means insert
				status.alignPos++;
			}
			return 0;
		}
			
		if (b0 == b1) { 
			for (int i = a0; i < a1; ++i) { 
				status.alignment[status.alignPos] = 2; // 2 means delete
				status.alignPos++;
			}
			return 0;
		}
		
		int n = a1 - a0;
		int m = b1 - b0;
		int splitRow = n / 2;
		
		double[][] D = new double[2][m+1];

		D[0][0] = 0;
		for (int j = 1; j <= m; ++j) 
			D[0][j] = D[0][j-1] + (B.get(b0 + (j-1)).weight() * params.penalty2);

		// Initialize crossing information.
		int[][] splitPoint = new int[2][m+1];
		int[][] exitPoint = new int[2][m+1];
		for (int j = 0; j <= m; ++j) { 
			splitPoint[0][j] = j;
			exitPoint[0][j] = j-1;
		}
		
		// Calculate the D array for the edit distance
		for (int i = 1; i <= n; ++i) {
			int modi = i%2;
			int modi1 = (i-1)%2;
			
			Symbol obj1 = A.get(a0 + (i-1));
			D[modi][0] = D[modi1][0] + (obj1.weight() * params.penalty1);

			splitPoint[modi][0] = 0;
			exitPoint[modi][0] = 0;

			for (int j = 1; j <= m; ++j) { 
				Symbol obj2 = B.get(b0 + (j-1));
				
				// Substitution will always be allowed and should have
				// Do not substitute if we have a complete mismatch though.
				double compare = params.similarity.similarity(obj1, obj2);
//				double compare = Symbol.tanimotoCoefficient(obj1, obj2);

				// if we have at least matched one thing....then it's not
				// a complete substitution
				double matchCost = D[modi1][j-1] + params.subPenalty;
				if (compare > 0) { 
					double v1 = obj1.weight();
					double v2 = obj2.weight();
					
					matchCost = D[modi1][j-1] + (params.bonus1*v1*compare) + (params.bonus2*v2*compare);
				}
				
				double insertCost = D[modi][j-1] + (params.penalty2 * obj2.weight());
				double deleteCost = D[modi1][j] + (params.penalty1 * obj1.weight());
								
				if (matchCost >= insertCost && matchCost >= deleteCost) { 
					D[modi][j] = matchCost;
					
					splitPoint[modi][j] = splitPoint[modi1][j-1];
					exitPoint[modi][j] = exitPoint[modi1][j-1];
					if (i==splitRow+1)
						exitPoint[modi][j] = j;
					
				} else if (insertCost > matchCost && insertCost >= deleteCost) { 
					D[modi][j] = insertCost;
					
					splitPoint[modi][j] = splitPoint[modi][j-1];
					exitPoint[modi][j] = exitPoint[modi][j-1];
					
				} else { 
					D[modi][j] = deleteCost;

					splitPoint[modi][j] = splitPoint[modi1][j];
					exitPoint[modi][j] = exitPoint[modi1][j];
					if (i==splitRow+1)
						exitPoint[modi][j] = j;
				}
			}

			// Set up check point if it is half way
			if (i==splitRow) {
				for (int k = 0;k <= m; ++k) {
					splitPoint[modi][k] = k;
					exitPoint[modi][k] = k-1; 
							// Assume exit is Insert until
							// determined on next pass
				}
			}
		} 

		double editDistance = D[n%2][m];

		int splitColumn = splitPoint[n%2][m]; // Where to finish top half
		int startPoint  = exitPoint[n%2][m];  // Where to start bottom half

		// Recurse for top half
//		logger.debug("--- recurse top half: " + a0 + " " + b0 + " " + splitRow + " " + splitColumn + " - " + startPoint);
		alignCheckp(params, A, B, a0, b0, a0+splitRow,  b0+splitColumn, status);

		// Now store alignment info. It was either a
		// delete or a match (mismatch), cause it had
		// to move to a new row.
		status.alignment[status.alignPos++] = (splitColumn==startPoint) ? 2 : 0;

		// Recurse for bottom half
//		logger.debug("--- recurse bottom half: " + a0 + " " + b0 + " " + splitRow + " " + startPoint);
		alignCheckp(params, A, B, a0+splitRow+1, b0+startPoint, a1, b1, status);

		return editDistance;		// Return edit distance
	}
	
	
	/**
	 * Calculate the distance
	 * @param params
	 * @return
	 */
	public static double distance(Params params) {
		List<Symbol> seq1 = subset(params.seq1, params.min1);
		List<Symbol> seq2 = subset(params.seq2, params.min2);

		int m = seq1.size();
		int n = seq2.size();

		double[] nextRow = new double[m + 1];
		double[] lastRow = new double[m + 1];
		lastRow[0] = 0.0D;
		for (int i = 1; i <= m; i++) {
			Symbol obj = seq1.get(i - 1);
			double previous = lastRow[(i - 1)];
			lastRow[i] = (previous + obj.weight() * params.penalty1);
		}

		double[] starterCol = new double[n + 1];
		starterCol[0] = 0.0D;
		for (int i = 1; i <= n; i++) {
			Symbol obj = seq2.get(i - 1);
			double previous = starterCol[(i - 1)];
			starterCol[i] = (previous + obj.weight() * params.penalty2);
		}

		for (int i = 1; i <= n; i++) {
			Symbol item2 = seq2.get(i - 1);
			nextRow[0] = starterCol[i];

			for (int j = 1; j <= m; j++) {
				Symbol item1 = seq1.get(j - 1);

				double compare = params.similarity.similarity(item1, item2);
				double choice1 = lastRow[(j - 1)] + params.subPenalty;
				if (compare > 0) {
					choice1 = lastRow[(j - 1)] 
					        + (params.bonus1*item1.weight()*compare)
							+ (params.bonus2*item2.weight()*compare);
				}

				double choice2 = lastRow[j] + item2.weight() * params.penalty2;
				double choice3 = nextRow[(j - 1)] + item1.weight() * params.penalty1;
				nextRow[j] = Math.max(choice1, Math.max(choice2, choice3));
			}

			double[] tmp = lastRow;
			lastRow = nextRow;
			nextRow = tmp;
		}

		double score = params.normalize.normalize(params, seq1, seq2, lastRow[m]);
		return score;
	}	
	
}

class Status { 
	public int[] alignment;
	public int alignPos;
}