package edu.arizona.cs.learn.algorithm.alignment;

import java.text.NumberFormat;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import org.apache.log4j.Logger;

import edu.arizona.cs.learn.timeseries.model.Instance;
import edu.arizona.cs.learn.timeseries.model.SequenceType;
import edu.arizona.cs.learn.timeseries.model.symbols.AllenRelation;
import edu.arizona.cs.learn.timeseries.model.symbols.StringSymbol;
import edu.arizona.cs.learn.timeseries.model.symbols.Symbol;
import edu.arizona.cs.learn.util.Utils;

public class TestAlignment {
	private static Logger logger = Logger.getLogger(TestAlignment.class);

	public static void main(String[] args) {
		test0();
		test1();
		test2();
		test3();
		test4();
		test5();
	}
	
	/**
	 * Write a string representation of the alignment.
	 * @param seq1
	 * @param seq2
	 * @return
	 */
	public static String toString(List<Symbol> seq1, List<Symbol> seq2) {
		return toString(seq1, seq2, true);
	}

	/**
	 * Write a string representation of the alignment 
	 * @param seq1
	 * @param seq2
	 * @param printMismatches
	 * @return
	 */
	public static String toString(List<Symbol> seq1, List<Symbol> seq2, 
			boolean printMismatches) {
		int longest = 0;
		for (int i = 0; i < seq1.size(); i++) {
			Symbol obj1 = seq1.get(i);
			if (obj1 != null) {
				String name = obj1.toString();
				longest = Math.max(name.length(), longest);
			}

			Symbol obj2 = seq2.get(i);
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
			Symbol obj1 = seq1.get(i);
			double objSize1 = 0.0D;
			if (obj1 != null) {
				objSize1 = obj1.weight();
			}
			buf.append(String.format(formatStr,
					new Object[] { Double.valueOf(objSize1), obj1 }));

			Symbol obj2 = seq2.get(i);
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

	private static void compareOutput(Params p) { 
		double d1 = GeneralAlignment.distance(p);
		
		Report r1 = GeneralAlignment.alignWithCons(p);
		Report r2 = GeneralAlignment.alignCheckp(p);
		
		
		logger.debug("Distances - " + d1 + " - " + r1.score + " - " + r2.score);
		
		String align1 = toString(r1.results1, r1.results2);
		String align2 = toString(r2.results1, r2.results2);
		
		boolean equal = align1.equals(align2);
		logger.debug("Alignments equal? - " + equal);
		if (equal) { 
			logger.debug("Alignment: \n" + align1);
		} else { 
			logger.debug("align1: \n" + align1);
			logger.debug("align2: \n" + align2);
		}
	}
	
	private static void test() {
		logger.debug("********* TEST ***********");
		Map<String,List<Instance>> map = Utils.load("ww3d-jump-over", SequenceType.allen);

		for (String s : map.keySet()) {
			logger.debug("Key: " + s);

			Params p = new Params();
			p.setMin(0, 0);
			p.setBonus(1.0D,1.0D);
			p.setPenalty(0.0D, 0.0D);

			List<Instance> list = map.get(s);
			for (int i = 0; i < list.size(); i++) {
				List<Symbol> seq1 = list.get(i).sequence();
				for (int j = i; j < list.size(); j++) {
					List<Symbol> seq2 = list.get(j).sequence();

					p.seq1 = seq1;
					p.seq2 = seq2;

					logger.debug("Distance[" + i + "," + j + "] : "
							+ GeneralAlignment.distance(p));
				}
			}
		}
	}
	
	private static void test0() { 
		logger.debug("********* TEST 0 ***********");
		List<Symbol> s1 = new ArrayList<Symbol>();
		s1.add(new StringSymbol("W"));
		s1.add(new StringSymbol("R"));
		s1.add(new StringSymbol("I"));
		s1.add(new StringSymbol("T"));
		s1.add(new StringSymbol("E"));
		s1.add(new StringSymbol("R"));
		s1.add(new StringSymbol("S"));

		List<Symbol> s2 = new ArrayList<Symbol>();
		s2.add(new StringSymbol("V"));
		s2.add(new StringSymbol("I"));
		s2.add(new StringSymbol("N"));
		s2.add(new StringSymbol("T"));
		s2.add(new StringSymbol("N"));
		s2.add(new StringSymbol("E"));
		s2.add(new StringSymbol("R"));

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

		List<Symbol> s1 = new ArrayList<Symbol>();
		s1.add(new AllenRelation("A", "ends-with", "D"));
		s1.add(new AllenRelation("A", "overlaps", "B", 5.0D));
		s1.add(new AllenRelation("A", "meets", "C", 5.0D));
		s1.add(new AllenRelation("D", "overlaps", "B"));
		s1.add(new AllenRelation("D", "overlaps", "C"));
		s1.add(new AllenRelation("B", "overlaps", "C", 5.0D));

		List<Symbol> s2 = new ArrayList<Symbol>();
		s2.add(new AllenRelation("C", "meets", "A"));
		s2.add(new AllenRelation("C", "before", "B"));
		s2.add(new AllenRelation("C", "before", "C"));
		s2.add(new AllenRelation("A", "overlaps", "B"));
		s2.add(new AllenRelation("A", "meets", "C"));
		s2.add(new AllenRelation("B", "overlaps", "C"));

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

		List<Symbol> s1 = new ArrayList<Symbol>();
		s1.add(new AllenRelation("A", "ends-with", "D"));
		s1.add(new AllenRelation("A", "overlaps", "B", 5.0D));
		s1.add(new AllenRelation("A", "meets", "C", 5.0D));
		s1.add(new AllenRelation("D", "overlaps", "B"));
		s1.add(new AllenRelation("D", "overlaps", "C"));
		s1.add(new AllenRelation("B", "overlaps", "C", 5.0D));

		List<Symbol> s2 = new ArrayList<Symbol>();
		s2.add(new AllenRelation("C", "meets", "A"));
		s2.add(new AllenRelation("C", "before", "B"));
		s2.add(new AllenRelation("C", "before", "C"));
		s2.add(new AllenRelation("A", "overlaps", "B"));
		s2.add(new AllenRelation("A", "meets", "C"));
		s2.add(new AllenRelation("B", "overlaps", "C"));

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

		List<Symbol> s1 = new ArrayList<Symbol>();
		s1.add(new StringSymbol("C"));
		s1.add(new StringSymbol("D"));
		s1.add(new StringSymbol("A", 5.0D));
		s1.add(new StringSymbol("Q", 3.0D));
		s1.add(new StringSymbol("R"));
		s1.add(new StringSymbol("S"));
		s1.add(new StringSymbol("B"));

		List<Symbol> s2 = new ArrayList<Symbol>();
		s2.add(new StringSymbol("A"));
		s2.add(new StringSymbol("B"));
		s2.add(new StringSymbol("A"));
		s2.add(new StringSymbol("T"));
		s2.add(new StringSymbol("Q"));
		s2.add(new StringSymbol("R"));
		s2.add(new StringSymbol("S"));

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

		List<Symbol> s1 = new ArrayList<Symbol>();
		s1.add(new AllenRelation("A", "ends-with", "D"));
		s1.add(new AllenRelation("A", "overlaps", "B"));
		s1.add(new AllenRelation("A", "meets", "C"));
		s1.add(new AllenRelation("D", "overlaps", "B"));
		s1.add(new AllenRelation("D", "overlaps", "C"));
		s1.add(new AllenRelation("B", "overlaps", "C"));

		List<Symbol> s2 = new ArrayList<Symbol>();
		s2.add(new AllenRelation("C", "meets", "A"));
		s2.add(new AllenRelation("C", "before", "B"));
		s2.add(new AllenRelation("C", "before", "C"));
		s2.add(new AllenRelation("A", "overlaps", "B"));
		s2.add(new AllenRelation("A", "meets", "C"));
		s2.add(new AllenRelation("B", "overlaps", "C"));

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

		List<Symbol> s1 = new ArrayList<Symbol>();
		s1.add(new StringSymbol("G"));
		s1.add(new StringSymbol("B"));
		s1.add(new StringSymbol("D"));
		s1.add(new StringSymbol("H"));
		s1.add(new StringSymbol("A"));

		List<Symbol> s2 = new ArrayList<Symbol>();
		s2.add(new StringSymbol("G"));
		s2.add(new StringSymbol("B"));
		s2.add(new StringSymbol("C"));
		s2.add(new StringSymbol("F"));
		s2.add(new StringSymbol("H"));
		s2.add(new StringSymbol("E"));
		s2.add(new StringSymbol("A"));

		Params p1 = new Params();
		p1.seq1 = s1;
		p1.seq2 = s2;
		p1.subPenalty = -2.0D;
		p1.setMin(0, 0);
		p1.setBonus(1.0D, 0.0D);
		p1.setPenalty(-1.0D, -1.0D);

		compareOutput(p1);
	}
}
