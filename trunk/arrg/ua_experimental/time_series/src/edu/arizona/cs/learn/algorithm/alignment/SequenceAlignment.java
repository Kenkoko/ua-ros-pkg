package edu.arizona.cs.learn.algorithm.alignment;

import java.io.File;
import java.io.RandomAccessFile;
import java.text.NumberFormat;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import org.apache.log4j.Logger;

import edu.arizona.cs.learn.algorithm.alignment.model.Cell;
import edu.arizona.cs.learn.algorithm.alignment.model.Cons;
import edu.arizona.cs.learn.algorithm.alignment.model.Instance;
import edu.arizona.cs.learn.algorithm.alignment.model.Symbol;
import edu.arizona.cs.learn.algorithm.alignment.model.WeightedObject;
import edu.arizona.cs.learn.timeseries.model.AllenRelation;
import edu.arizona.cs.learn.util.RandomFile;
import edu.arizona.cs.learn.util.SequenceType;
import edu.arizona.cs.learn.util.Utils;

public class SequenceAlignment {
	private static Logger logger;

	static {
		logger = Logger.getLogger(SequenceAlignment.class);
	}

	/**
	 * Mash together to the two sequences and create one sequence that is 
	 * increments weights where the objects match and inserts new items where they
	 * don't match.
	 * @param seq1
	 * @param seq2
	 * @return
	 */
	public static List<WeightedObject> combineAlignments(List<WeightedObject> seq1, List<WeightedObject> seq2) {
		assert (seq1.size() == seq2.size());
		List<WeightedObject> results = new ArrayList<WeightedObject>();
		for (int i = 0; i < seq1.size(); i++) {
			WeightedObject item1 = (WeightedObject) seq1.get(i);
			WeightedObject item2 = (WeightedObject) seq2.get(i);
			if (item1 != null) {
				if (item2 != null)
					item1.increment(item2.weight());
				results.add(item1);
			} else {
				results.add(item2.copy());
			}
		}
		return results;
	}

	/**
	 * Write a string representation of the alignment.
	 * @param seq1
	 * @param seq2
	 * @return
	 */
	public static String toString(List<WeightedObject> seq1, List<WeightedObject> seq2) {
		return toString(seq1, seq2, true);
	}

	/**
	 * Write a string representation of the alignment 
	 * @param seq1
	 * @param seq2
	 * @param printMismatches
	 * @return
	 */
	public static String toString(List<WeightedObject> seq1, List<WeightedObject> seq2, 
			boolean printMismatches) {
		int longest = 0;
		for (int i = 0; i < seq1.size(); i++) {
			WeightedObject obj1 = (WeightedObject) seq1.get(i);
			if (obj1 != null) {
				String name = obj1.toString();
				longest = Math.max(name.length(), longest);
			}

			WeightedObject obj2 = (WeightedObject) seq2.get(i);
			if (obj2 != null) {
				String name = obj2.toString();
				longest = Math.max(name.length(), longest);
			}
		}

		longest = Math.max(longest, 4);

		NumberFormat nf = NumberFormat.getInstance();
		nf.setMinimumFractionDigits(4);
		nf.setMaximumFractionDigits(4);
		nf.setMinimumIntegerDigits(1);
		nf.setMaximumIntegerDigits(1);

		String formatStr = "%1$2s %2$" + longest + "s ";

		StringBuffer buf = new StringBuffer("\n");
		for (int i = 0; i < seq1.size(); i++) {
			if ((!printMismatches)
					&& ((seq1.get(i) == null) || (seq2.get(i) == null))) {
				continue;
			}
			WeightedObject obj1 = (WeightedObject) seq1.get(i);
			double objSize1 = 0.0D;
			if (obj1 != null) {
				objSize1 = obj1.weight();
			}
			buf.append(String.format(formatStr,
					new Object[] { Double.valueOf(objSize1), obj1 }));

			WeightedObject obj2 = (WeightedObject) seq2.get(i);
			double objSize2 = 0.0D;
			if (obj2 != null) {
				objSize2 = obj2.weight();
			}
			buf.append(String.format(formatStr,
					new Object[] { Double.valueOf(objSize2), obj2 }));
			buf.append("\n");
		}
		return buf.toString();
	}

	/**
	 * Select a subset of the objects in the sequences.  Specifically
	 * those that have been seen at least minSeen times.
	 * @param seq
	 * @param minSeen
	 * @return
	 */
	public static List<WeightedObject> subset(List<WeightedObject> seq, int minSeen) {
		if (minSeen == 0) 
			return seq;

		List<WeightedObject> seq1 = new ArrayList<WeightedObject>();
		double seenCount = 0.0D;
		for (WeightedObject obj : seq) {
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
	 * Normalize the distance metric
	 * @param params
	 * @param seq1
	 * @param seq2
	 * @param score
	 * @return
	 */
	private static double normalizeScore(Params params, 
			List<WeightedObject> seq1, List<WeightedObject> seq2, 
			double score) {
		
		double worstScore = 0.0D;
		double bestScore = 0.0D;
		for (WeightedObject obj : seq1) {
			bestScore += params.bonus1 * obj.weight();
			worstScore += params.penalty1 * obj.weight();
			worstScore += params.gapPenalty;
		}

		for (WeightedObject obj : seq2) {
			bestScore += params.bonus2 * obj.weight();
			worstScore += params.penalty2 * obj.weight();
			worstScore += params.gapPenalty;
		}
		worstScore = Math.abs(worstScore);

		return 1.0D - (score + worstScore) / (bestScore + worstScore);
	}

	/**
	 * Normalize the distance metric assuming that we are 
	 * using it for kNN (LCSS)
	 * @param params
	 * @param seq1
	 * @param seq2
	 * @param score
	 * @return
	 */
	public static double normalizeKNN(Params params, List<WeightedObject> seq1,
			List<WeightedObject> seq2, double score) {
		double min = Math.min(seq1.size(), seq2.size());
		double max = Math.max(seq1.size(), seq2.size());
		double sum = seq1.size() + seq2.size();
		double maxScore = sum - (max - min);

		return 1.0D - score / maxScore;
	}

	/**
	 * Perform the sequence alignment.
	 * @param params
	 * @return
	 */
	public static Report align(Params params) {
		// now we will be using the linear space algorithm
		return alignCheckp(params);
		
//		if ((params.seq1.size() > 10000) || (params.seq2.size() > 10000)) {
//			return alignWithFile(params);
//		}
//		return alignWithCons(params);
	}

	/**
	 * Perform the sequence alignment building the table in 
	 * main memory.
	 * @param params
	 * @return
	 */
	private static Report alignWithCons(Params params) {
		boolean printTable = false;

		List<WeightedObject> seq1 = subset(params.seq1, params.min1);
		List<WeightedObject> seq2 = subset(params.seq2, params.min2);

		int m = seq1.size();
		int n = seq2.size();

		Cell[] nextRow = new Cell[m + 1];
		Cell[] lastRow = new Cell[m + 1];
		lastRow[0] = new Cell(0.0D);

		if (printTable)
			System.out.print("0 & ");
		for (int i = 1; i <= m; i++) {
			WeightedObject obj = (WeightedObject) seq1.get(i - 1);
			Cell previous = lastRow[(i - 1)];

			double value = previous.score + obj.weight() * params.penalty1
					+ params.gapPenalty;

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
			WeightedObject obj = (WeightedObject) seq2.get(i - 1);
			Cell previous = starterCol[(i - 1)];

			double value = previous.score + obj.weight() * params.penalty2
					+ params.gapPenalty;

			Cell next = new Cell(value);
			next.directions = new Cons('u', previous.directions);
			starterCol[i] = next;
		}

		for (int i = 1; i <= n; i++) {
			WeightedObject item2 = (WeightedObject) seq2.get(i - 1);

			nextRow[0] = starterCol[i];

			if (printTable)
				System.out.print(nextRow[0].score + " & ");
			for (int j = 1; j <= m; j++) {
				WeightedObject item1 = (WeightedObject) seq1.get(j - 1);

				Cell diag = lastRow[(j - 1)];
				Cell up = lastRow[j];
				Cell left = nextRow[(j - 1)];

				double choice1 = diag.score + params.subPenalty;
				if (item1.equals(item2)) {
					double v1 = item1.weight();
					double v2 = item2.weight();
					choice1 = diag.score + params.bonus1 * v1 + params.bonus2
							* v2;
				}

				double choice2 = up.score + item2.weight() * params.penalty2
						+ params.gapPenalty;
				double choice3 = left.score + item1.weight() * params.penalty1
						+ params.gapPenalty;

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

		// now we need to normalize based on what is given in the parameeters
		switch (params.normalize) { 
		case knn:
			sc.score = normalizeKNN(params, seq1, seq2, best.score);
			break;
		case signature:
		case regular:
			sc.score = normalizeScore(params, seq1, seq2, best.score);
			break;
		case none:
			sc.score = best.score;
		}

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
	 * Perform the alignment but keep the table as a RandomAccessFile
	 * @param params
	 * @return
	 */
	private static Report alignWithFile(Params params) {
		File f = null;
		try {
			Runtime r = Runtime.getRuntime();
			long startMemory = r.totalMemory() - r.freeMemory();
			long startTime = System.currentTimeMillis();

			f = RandomFile.sequenceAlignmentFile();
			if (f.exists())
				f.delete();
			f.createNewFile();

			RandomAccessFile file = new RandomAccessFile(f, "rw");

			List<WeightedObject> seq1 = subset(params.seq1, params.min1);
			List<WeightedObject> seq2 = subset(params.seq2, params.min2);

			int m = seq1.size();
			int n = seq2.size();
			logger.debug("(m x n) - (" + m + "," + n + ")");

			double[] nextRow = new double[m + 1];
			double[] lastRow = new double[m + 1];
			lastRow[0] = 0.0D;

			byte[] b = new byte[m + 1];
			b[0] = 1;
			for (int i = 1; i <= m; i++) {
				WeightedObject obj = (WeightedObject) seq1.get(i - 1);
				double previous = lastRow[(i - 1)];
				lastRow[i] = (previous + obj.weight() * params.penalty1 + params.gapPenalty);
				b[i] = 2;
			}
			file.write(b);

			double[] starterCol = new double[n + 1];
			starterCol[0] = 0.0D;
			for (int i = 1; i <= n; i++) {
				WeightedObject obj = (WeightedObject) seq2.get(i - 1);
				double previous = starterCol[(i - 1)];
				starterCol[i] = (previous + obj.weight() * params.penalty2 + params.gapPenalty);
			}

			for (int i = 1; i <= n; i++) {
				WeightedObject item2 = (WeightedObject) seq2.get(i - 1);
				nextRow[0] = starterCol[i];
				b[0] = 0;

				for (int j = 1; j <= m; j++) {
					WeightedObject item1 = (WeightedObject) seq1.get(j - 1);

					double choice1 = lastRow[(j - 1)] + params.subPenalty;
					if (item1.equals(item2)) {
						double v1 = item1.weight();
						double v2 = item2.weight();
						choice1 = lastRow[(j - 1)] + params.bonus1 * v1
								+ params.bonus2 * v2;
					}

					double choice2 = lastRow[j] + item2.weight()
							* params.penalty2 + params.gapPenalty;
					double choice3 = nextRow[(j - 1)] + item1.weight()
							* params.penalty1 + params.gapPenalty;

					if ((choice1 >= choice2) && (choice1 >= choice3))
						b[j] = 1;
					else if ((choice2 >= choice3) && (choice2 > choice1))
						b[j] = 0;
					else if ((choice3 > choice2) && (choice3 > choice1))
						b[j] = 2;
					else {
						throw new RuntimeException("Weird: " + choice1 + " "
								+ choice2 + " " + choice3);
					}
					nextRow[j] = Math.max(choice1, Math.max(choice2, choice3));
				}
				double[] tmp = lastRow;
				lastRow = nextRow;
				nextRow = tmp;

				file.write(b);
			}

			Report sc = new Report();
			sc.s1Size = m;
			sc.s2Size = n;

			// now we need to normalize based on what is given in the parameeters
			switch (params.normalize) { 
			case knn:
				sc.score = normalizeKNN(params, seq1, seq2, lastRow[m]);
				break;
			case signature:
			case regular:
				sc.score = normalizeScore(params, seq1, seq2, lastRow[m]);
				break;
			case none:
				sc.score = lastRow[m];
			}

			int i = m;
			int j = n;
			long tmpM = m;
			do {
				long charIndex = j * (tmpM + 1L) + i;
				if (charIndex < 0L) {
					logger.error("We've wrapped -- m: " + m + " -- n: " + n);
				}
				file.seek(charIndex);
				byte direction = file.readByte();

				switch (direction) {
				case 1:
					sc.add(seq1.get(i - 1), seq2.get(j - 1));
					i--;
					j--;
					break;
				case 2:
					sc.add(seq1.get(i - 1), null);
					i--;
					break;
				case 0:
					sc.add(null, seq2.get(j - 1));
					j--;
				}
				if (i <= 0)
					break;
			} while (j > 0);

			while (j > 0) {
				sc.add(null, seq2.get(j - 1));
				j--;
			}

			while (i > 0) {
				sc.add(seq1.get(i - 1), null);
				i--;
			}

			assert ((i == 0) && (j == 0));
			sc.finish();

			file.close();
			f.delete();

			long endMemory = r.totalMemory() - r.freeMemory();
			long endTime = System.currentTimeMillis();

			NumberFormat nf = NumberFormat.getInstance();
			nf.setGroupingUsed(false);
			nf.setMinimumFractionDigits(2);
			nf.setMaximumFractionDigits(2);

			double elapsedTime = (endTime - startTime) / 60000.0D;
			String elapsed = nf.format(elapsedTime);

			if (elapsedTime > 1.0D)
				logger.debug("\ttraining elapsed " + elapsed + " mins");
			Report localReport1 = sc;
			return localReport1;
		} catch (Exception e) {
			throw new RuntimeException(e.getMessage());
		} finally {
			if ((f != null) && (f.exists()))
				f.delete();
		}
	}
	
	/**
	 * Code structure courtesy of David Powell
	 * 	Time Complexity: O(n*n)  Space Complexity: O(n) 
	 * @param params
	 * @return
	 */
	public static Report alignCheckp(Params params) { 
		List<WeightedObject> A = subset(params.seq1, params.min1);
		List<WeightedObject> B = subset(params.seq2, params.min2);
		
//		logger.debug("Sizes: " + A.size() + " " + B.size());
		Status status = new Status();
		status.alignment = new int[params.seq1.size() + params.seq2.size() + 1];
		status.alignPos = 0;

		double distance = alignCheckp(params, A, B, 0, 0, A.size(), B.size(), status);
		switch (params.normalize) { 
		case knn:
			distance = normalizeKNN(params, A, B, distance);
			break;
		case signature:
		case regular:
			distance = normalizeScore(params, A, B, distance);
			break;
		}

		Report report = new Report();
		report.s1Size = A.size();
		report.s2Size = B.size();
		report.score = distance;
		
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
			List<WeightedObject> A, List<WeightedObject> B,
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
			
			WeightedObject obj1 = A.get(a0 + (i-1));
			D[modi][0] = D[modi1][0] + (obj1.weight() * params.penalty1);

			splitPoint[modi][0] = 0;
			exitPoint[modi][0] = 0;

			for (int j = 1; j <= m; ++j) { 
				WeightedObject obj2 = B.get(b0 + (j-1));
				
				double matchCost = D[modi1][j-1] + params.subPenalty;
				if (obj1.equals(obj2)) { 
					double v1 = obj1.weight();
					double v2 = obj2.weight();
					matchCost = D[modi1][j-1] + (params.bonus1*v1) + (params.bonus2*v2);
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
		List<WeightedObject> seq1 = subset(params.seq1, params.min1);
		List<WeightedObject> seq2 = subset(params.seq2, params.min2);

		int m = seq1.size();
		int n = seq2.size();

		double[] nextRow = new double[m + 1];
		double[] lastRow = new double[m + 1];
		lastRow[0] = 0.0D;
		for (int i = 1; i <= m; i++) {
			WeightedObject obj = seq1.get(i - 1);
			double previous = lastRow[(i - 1)];
			lastRow[i] = (previous + obj.weight() * params.penalty1 + params.gapPenalty);
		}

		double[] starterCol = new double[n + 1];
		starterCol[0] = 0.0D;
		for (int i = 1; i <= n; i++) {
			WeightedObject obj = seq2.get(i - 1);
			double previous = starterCol[(i - 1)];
			starterCol[i] = (previous + obj.weight() * params.penalty2 + params.gapPenalty);
		}

		for (int i = 1; i <= n; i++) {
			WeightedObject item2 = seq2.get(i - 1);
			nextRow[0] = starterCol[i];

			for (int j = 1; j <= m; j++) {
				WeightedObject item1 = seq1.get(j - 1);

				double choice1 = lastRow[(j - 1)] + params.subPenalty;
				if (item1.equals(item2)) {
					choice1 = lastRow[(j - 1)] + params.bonus1 * item1.weight()
							+ params.bonus2 * item2.weight();
				}

				double choice2 = lastRow[j] + item2.weight() * params.penalty2
						+ params.gapPenalty;
				double choice3 = nextRow[(j - 1)] + item1.weight()
						* params.penalty1 + params.gapPenalty;
				nextRow[j] = Math.max(choice1, Math.max(choice2, choice3));
			}

			double[] tmp = lastRow;
			lastRow = nextRow;
			nextRow = tmp;
		}
		double score = lastRow[m];
		// now we need to normalize based on what is given in the parameeters
		switch (params.normalize) { 
		case knn:
			score = normalizeKNN(params, seq1, seq2, score);
			break;
		case signature:
		case regular:
			score = normalizeScore(params, seq1, seq2, score);
			break;
		case none:
			break;
		}

		return score;
	}

	/**
	 * Longest-common-subsequence.  
	 * @param seq1
	 * @param seq2
	 * @return
	 */
	public static double lcs(List<WeightedObject> seq1, List<WeightedObject> seq2) {
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
			WeightedObject item2 = (WeightedObject) seq2.get(i - 1);
			nextRow[0] = starterCol[i];

			for (int j = 1; j <= m; j++) {
				WeightedObject item1 = (WeightedObject) seq1.get(j - 1);

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

	public static void main(String[] args) {
		test0();
		test1();
		test2();
		test3();
		test4();
		test5();
	}

	private static void compareOutput(Params p) { 
		double d1 = distance(p);
		
		Report r1 = alignWithCons(p);
		Report r2 = alignWithFile(p);
		Report r3 = alignCheckp(p);
		
		logger.debug("Distances - " + d1 + " - " + r1.score + " - " + r2.score + " - " + r3.score);
		
		String align1 = toString(r1.results1, r1.results2);
		String align2 = toString(r2.results1, r2.results2);
		String align3 = toString(r3.results1, r3.results2);
		
		boolean equal = align1.equals(align2) && align1.equals(align3);
		logger.debug("Alignments equal? - " + equal);
		if (equal) { 
			logger.debug("Alignment: \n" + align1);
		} else { 
			logger.debug("align1: \n" + align1);
			logger.debug("align2: \n" + align2);
			logger.debug("align3: \n" + align3);
		}
	}
	
	private static void test() {
		logger.debug("********* TEST ***********");
		Map<String,List<Instance>> map = Utils.load("ww3d-jump-over", SequenceType.allen);

		for (String s : map.keySet()) {
			logger.debug("Key: " + s);

			Params p = new Params();
			p.setMin(0, 0);
			p.setBonus(1.0D, 1.0D);
			p.setPenalty(0.0D, 0.0D);

			List<Instance> list = map.get(s);
			for (int i = 0; i < list.size(); i++) {
				List<WeightedObject> seq1 = list.get(i).sequence();
				for (int j = i; j < list.size(); j++) {
					List<WeightedObject> seq2 = list.get(j).sequence();

					p.seq1 = seq1;
					p.seq2 = seq2;

					logger.debug("Distance[" + i + "," + j + "] : "
							+ distance(p));
				}
			}
		}
	}
	
	private static void test0() { 
		logger.debug("********* TEST 0 ***********");
		List<WeightedObject> s1 = new ArrayList<WeightedObject>();
		s1.add(new WeightedObject(new Symbol("W"), 1));
		s1.add(new WeightedObject(new Symbol("R"), 1));
		s1.add(new WeightedObject(new Symbol("I"), 1));
		s1.add(new WeightedObject(new Symbol("T"), 1));
		s1.add(new WeightedObject(new Symbol("E"), 1));
		s1.add(new WeightedObject(new Symbol("R"), 1));
		s1.add(new WeightedObject(new Symbol("S"), 1));

		List<WeightedObject> s2 = new ArrayList<WeightedObject>();
		s2.add(new WeightedObject(new Symbol("V"), 1));
		s2.add(new WeightedObject(new Symbol("I"), 1));
		s2.add(new WeightedObject(new Symbol("N"), 1));
		s2.add(new WeightedObject(new Symbol("T"), 1));
		s2.add(new WeightedObject(new Symbol("N"), 1));
		s2.add(new WeightedObject(new Symbol("E"), 1));
		s2.add(new WeightedObject(new Symbol("R"), 1));

		Params p1 = new Params();
		p1.setMin(0, 0);
		p1.setBonus(1.0D, 1.0D);
		p1.setPenalty(0, 0.0D);

		p1.seq1 = s1;
		p1.seq2 = s2;
		
		compareOutput(p1);
	}

	private static void test1() {
		logger.debug("********* TEST 1 ***********");

		List<WeightedObject> s1 = new ArrayList<WeightedObject>();
		s1.add(new WeightedObject(new AllenRelation("A", "ends-with", "D"), 1.0D));
		s1.add(new WeightedObject(new AllenRelation("A", "overlaps", "B"), 5.0D));
		s1.add(new WeightedObject(new AllenRelation("A", "meets", "C"), 5.0D));
		s1.add(new WeightedObject(new AllenRelation("D", "overlaps", "B"), 1.0D));
		s1.add(new WeightedObject(new AllenRelation("D", "overlaps", "C"), 1.0D));
		s1.add(new WeightedObject(new AllenRelation("B", "overlaps", "C"), 5.0D));

		List<WeightedObject> s2 = new ArrayList<WeightedObject>();
		s2.add(new WeightedObject(new AllenRelation("C", "meets", "A"), 1.0D));
		s2.add(new WeightedObject(new AllenRelation("C", "before", "B"), 1.0D));
		s2.add(new WeightedObject(new AllenRelation("C", "before", "C"), 1.0D));
		s2.add(new WeightedObject(new AllenRelation("A", "overlaps", "B"), 1.0D));
		s2.add(new WeightedObject(new AllenRelation("A", "meets", "C"), 1.0D));
		s2.add(new WeightedObject(new AllenRelation("B", "overlaps", "C"), 1.0D));

		Params p1 = new Params();
		p1.setMin(0, 0);
		p1.setBonus(1.0D, 0.0D);
		p1.setPenalty(-1.0D, 0.0D);

		p1.seq1 = s1;
		p1.seq2 = s2;

		compareOutput(p1);
	}

	private static void test2() {
		logger.debug("********* TEST 2 ***********");

		List<WeightedObject> s1 = new ArrayList<WeightedObject>();
		s1.add(new WeightedObject(new AllenRelation("A", "ends-with", "D"), 1.0D));
		s1.add(new WeightedObject(new AllenRelation("A", "overlaps", "B"), 5.0D));
		s1.add(new WeightedObject(new AllenRelation("A", "meets", "C"), 5.0D));
		s1.add(new WeightedObject(new AllenRelation("D", "overlaps", "B"), 1.0D));
		s1.add(new WeightedObject(new AllenRelation("D", "overlaps", "C"), 1.0D));
		s1.add(new WeightedObject(new AllenRelation("B", "overlaps", "C"), 5.0D));

		List<WeightedObject> s2 = new ArrayList<WeightedObject>();
		s2.add(new WeightedObject(new AllenRelation("C", "meets", "A"), 1.0D));
		s2.add(new WeightedObject(new AllenRelation("C", "before", "B"), 1.0D));
		s2.add(new WeightedObject(new AllenRelation("C", "before", "C"), 1.0D));
		s2.add(new WeightedObject(new AllenRelation("A", "overlaps", "B"), 1.0D));
		s2.add(new WeightedObject(new AllenRelation("A", "meets", "C"), 1.0D));
		s2.add(new WeightedObject(new AllenRelation("B", "overlaps", "C"), 1.0D));

		Params p1 = new Params();
		p1.seq1 = s1;
		p1.seq2 = s2;
		p1.min1 = 0;
		p1.min2 = 0;
		p1.bonus1 = 1.0D;
		p1.bonus2 = 0.0D;
		p1.penalty1 = -1.0D;
		p1.penalty2 = 0.0D;

		compareOutput(p1);
	}

	public static void test3() {
		logger.debug("********* TEST 3 ***********");

		List<WeightedObject> s1 = new ArrayList<WeightedObject>();
		s1.add(new WeightedObject(new Symbol("C"), 1.0D));
		s1.add(new WeightedObject(new Symbol("D"), 1.0D));
		s1.add(new WeightedObject(new Symbol("A"), 5.0D));
		s1.add(new WeightedObject(new Symbol("Q"), 3.0D));
		s1.add(new WeightedObject(new Symbol("R"), 1.0D));
		s1.add(new WeightedObject(new Symbol("S"), 1.0D));
		s1.add(new WeightedObject(new Symbol("B"), 1.0D));

		List<WeightedObject> s2 = new ArrayList<WeightedObject>();
		s2.add(new WeightedObject(new Symbol("A"), 1.0D));
		s2.add(new WeightedObject(new Symbol("B"), 1.0D));
		s2.add(new WeightedObject(new Symbol("A"), 1.0D));
		s2.add(new WeightedObject(new Symbol("T"), 1.0D));
		s2.add(new WeightedObject(new Symbol("Q"), 1.0D));
		s2.add(new WeightedObject(new Symbol("R"), 1.0D));
		s2.add(new WeightedObject(new Symbol("S"), 1.0D));

		Params p1 = new Params();
		p1.seq1 = s1;
		p1.seq2 = s2;
		p1.min1 = 0;
		p1.min2 = 0;
		p1.bonus1 = 1.0D;
		p1.bonus2 = 0.0D;
		p1.penalty1 = 0.0D;
		p1.penalty2 = 0.0D;
		
		compareOutput(p1);
	}

	private static void test4() {
		logger.debug("********* TEST 4 ***********");

		List<WeightedObject> s1 = new ArrayList<WeightedObject>();
		s1.add(new WeightedObject(new AllenRelation("A", "ends-with", "D"), 1.0D));
		s1.add(new WeightedObject(new AllenRelation("A", "overlaps", "B"), 1.0D));
		s1.add(new WeightedObject(new AllenRelation("A", "meets", "C"), 1.0D));
		s1.add(new WeightedObject(new AllenRelation("D", "overlaps", "B"), 1.0D));
		s1.add(new WeightedObject(new AllenRelation("D", "overlaps", "C"), 1.0D));
		s1.add(new WeightedObject(new AllenRelation("B", "overlaps", "C"), 1.0D));

		List<WeightedObject> s2 = new ArrayList<WeightedObject>();
		s2.add(new WeightedObject(new AllenRelation("C", "meets", "A"), 1.0D));
		s2.add(new WeightedObject(new AllenRelation("C", "before", "B"), 1.0D));
		s2.add(new WeightedObject(new AllenRelation("C", "before", "C"), 1.0D));
		s2.add(new WeightedObject(new AllenRelation("A", "overlaps", "B"), 1.0D));
		s2.add(new WeightedObject(new AllenRelation("A", "meets", "C"), 1.0D));
		s2.add(new WeightedObject(new AllenRelation("B", "overlaps", "C"), 1.0D));

		Params p1 = new Params();
		p1.setMin(0, 0);
		p1.setBonus(1.0D, 0.0D);
		p1.setPenalty(0.0D, 0.0D);
		p1.seq1 = s1;
		p1.seq2 = s2;
		
		compareOutput(p1);
	}

	public static void test5() {
		logger.debug("********* TEST 5 ***********");

		List<WeightedObject> s1 = new ArrayList<WeightedObject>();
		s1.add(new WeightedObject(new Symbol("G"), 1.0D));
		s1.add(new WeightedObject(new Symbol("B"), 1.0D));
		s1.add(new WeightedObject(new Symbol("D"), 1.0D));
		s1.add(new WeightedObject(new Symbol("H"), 1.0D));
		s1.add(new WeightedObject(new Symbol("A"), 1.0D));

		List<WeightedObject> s2 = new ArrayList<WeightedObject>();
		s2.add(new WeightedObject(new Symbol("G"), 1.0D));
		s2.add(new WeightedObject(new Symbol("B"), 1.0D));
		s2.add(new WeightedObject(new Symbol("C"), 1.0D));
		s2.add(new WeightedObject(new Symbol("F"), 1.0D));
		s2.add(new WeightedObject(new Symbol("H"), 1.0D));
		s2.add(new WeightedObject(new Symbol("E"), 1.0D));
		s2.add(new WeightedObject(new Symbol("A"), 1.0D));

		Params p1 = new Params();
		p1.seq1 = s1;
		p1.seq2 = s2;
		p1.gapPenalty = 0.0D;
		p1.subPenalty = -2.0D;
		p1.setMin(0, 0);
		p1.setBonus(1.0D, 0.0D);
		p1.setPenalty(-1.0D, -1.0D);

		compareOutput(p1);
	}
}

class Status { 
	public int[] alignment;
	public int alignPos;
}
