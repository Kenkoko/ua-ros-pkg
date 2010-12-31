package edu.arizona.cs.learn.algorithm.alignment;

import java.text.NumberFormat;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import edu.arizona.cs.learn.timeseries.model.symbols.Symbol;


public class Report {

	public double score;
	
	public int numMatches;
	public int s1Mismatch;
	public int s2Mismatch;
	
	public int s1Size;
	public int s2Size;
	
	public List<Symbol> results1;
	public List<Symbol> results2;
	
	public Report() { 
		results1 = new ArrayList<Symbol>();
		results2 = new ArrayList<Symbol>();
	}
	
	public Report(double score) { 
		this();
		
		this.score = score;
	}
	
	public void add(Symbol obj1, Symbol obj2) { 
		results1.add(obj1);
		results2.add(obj2);
		
		if (obj1 == null)
			++s2Mismatch;
		if (obj2 == null)
			++s1Mismatch;
		if (obj1 != null && obj2 != null) {
			++numMatches;
		}
	}
	
	public void copy(Report sc) { 
		results1 = new ArrayList<Symbol>(sc.results1);
		results2 = new ArrayList<Symbol>(sc.results2);
		
		s1Mismatch = sc.s1Mismatch;
		s2Mismatch = sc.s2Mismatch;
	}
	
	public void assign(Report sc) { 
		results1 = sc.results1;
		results2 = sc.results2;

		s1Mismatch = sc.s1Mismatch;
		s2Mismatch = sc.s2Mismatch;
	}
	
	public void finish() { 
		Collections.reverse(results1);
		Collections.reverse(results2);
	}
	
	public String toString() { 
		return score + " " + results1.size() + " " + results2.size() + " --- " + numMatches;
	}
	
	public void print() { 
		System.out.println("Score: " + score);
		
		int longest = 0;
		for (int i = 0; i < results1.size(); i++) {
			Symbol obj1 = results1.get(i);
			if (obj1 != null) {
				String name = obj1.toString();
				longest = Math.max(name.length(), longest);
			}

			Symbol obj2 = results2.get(i);
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
		for (int i = 0; i < results1.size(); i++) {
			Symbol obj1 = results1.get(i);
			double objSize1 = 0.0D;
			if (obj1 != null) {
				objSize1 = obj1.weight();
			}
			buf.append(String.format(formatStr,
					new Object[] { Double.valueOf(objSize1), obj1 }));

			Symbol obj2 = results2.get(i);
			double objSize2 = 0.0D;
			if (obj2 != null) {
				objSize2 = obj2.weight();
			}
			buf.append(String.format(formatStr,
					new Object[] { Double.valueOf(objSize2), obj2 }));
			
			buf.append("\n");
		}
		System.out.println(buf.toString());
	}
}
