package edu.arizona.cs.learn.timeseries.dissertation;

import java.util.List;
import java.util.Map;

import edu.arizona.cs.learn.algorithm.alignment.GeneralAlignment;
import edu.arizona.cs.learn.algorithm.alignment.Params;
import edu.arizona.cs.learn.timeseries.model.Instance;
import edu.arizona.cs.learn.timeseries.model.SequenceType;
import edu.arizona.cs.learn.timeseries.model.symbols.Symbol;
import edu.arizona.cs.learn.util.Utils;

public class SequenceAlignmentTable {
	private String eol = "\\\\ \\hhline{~--";
	private String mc  = "\\mc{1}{c|}{ins}";
	private String mctop  = "\\mc{1}{c}{ins}";
	private String mc0 = "\\mc{1}{c}{}";
	private String mc1 = "\\mc{1}{c|}{}";
	private String cc1 = "\\cellcolor[gray]{0.8}";
	private String cc2 = "\\cellcolor[gray]{0.8}";
	private String d = "{\\tiny $\\nwarrow$}";
	private String l = "{\\tiny $\\leftarrow$}";
	private String u = "{\\tiny $\\uparrow$}";

	public SequenceAlignmentTable() { 
	}
	
	public void latexTable(Params params) { 
		StringBuilder buf = new StringBuilder("\\begin{tabular}{|c|c|c|");
		
		List<Symbol> seq1 = GeneralAlignment.subset(params.seq1, params.min1);
		List<Symbol> seq2 = GeneralAlignment.subset(params.seq2, params.min2);
		
		int m = seq1.size();
		int n = seq2.size();

//		logger.debug("s1: " + m + " s2: " + n);
		
		char[] row = new char[m+1];
		row[0] = 'u';
		
		double[] nextRow = new double[m+1];
		double[] lastRow = new double[m+1];
		lastRow[0] = 0;
		for (int i = 1; i <= m; ++i) {
			// this should be running sum of the cost of this sequence
			Symbol obj = seq1.get(i-1);
			double previous = lastRow[i-1];
			lastRow[i] = previous + (obj.weight()*params.penalty1);
		}

		double[] starterCol = new double[n+1];
		starterCol[0] = 0;
		for (int i = 1; i <= n; ++i) {
			// this should be running sum of the cost of this sequence
			Symbol obj = seq2.get(i-1);
			double previous = starterCol[i-1];
			starterCol[i] = previous + (obj.weight()*params.penalty2);
		}		
		
		// begin constructing the table string.... 
		for (int i = 0; i < params.seq1.size(); ++i) { 
			buf.append("c|");
			eol += "-";
		}
		buf.append("}\n");
		eol += "}\n";
		// first row is the indexes....
		buf.append(mc0 + " & " + mc0);
		for (int i = 0; i <= seq1.size(); ++i) { 
			buf.append(" & " + mctop.replaceAll("ins", i+""));
		}
		buf.append(eol);
		
		// second row is the actual sequence values
		buf.append(mc1 + " & " + cc1 + " & " + cc1 + " $\\emptyset$ ");
		for (Symbol obj : seq1) { 
			buf.append(" & " + cc1 + " \\prop{" + obj.toString() + "} ");
		}
		buf.append(eol);
		
		// third and fourth row is the empty set and first row
		buf.append(mc.replaceAll("ins", 0+"") + " & " + cc1 + " $\\emptyset$ & " + cc2 + " 0");
		for (int i = 0; i < seq1.size(); ++i) { 
			buf.append(" & " + l + " " + ((i+1)*-1));
		}
		buf.append(eol);
		
		for (int i = 1; i <= n; ++i) { 
			Symbol item2 = seq2.get(i-1);
			nextRow[0] = starterCol[i];

			for (int j = 1; j <= m; ++j) { 
				Symbol item1 = seq1.get(j-1);

				double choice1 = lastRow[j-1] + params.subPenalty;
				if (item1.equals(item2)) {
					choice1 = lastRow[j-1] +
							(params.bonus1 * item1.weight()) + 
							(params.bonus2 * item2.weight());
				}

				double choice2 = lastRow[j] + (item2.weight()*params.penalty2);
				double choice3 = nextRow[j-1] + (item1.weight()*params.penalty1);
				nextRow[j] = Math.max(choice1, Math.max(choice2, choice3));
				
				if (choice1 >= choice2 && choice1 >= choice3) { 
					row[j] = 'd';
				} else if (choice2 >= choice3 && choice2 > choice1) { 
					row[j] = 'u';
				} else if (choice3 > choice2 && choice3 > choice1) {
					row[j] = 'l';
				} else { 
					throw new RuntimeException("Weird: [" + i + "," + j + "] " + item1.weight() + " " + item2.weight() + " " +
							choice1 + " " + choice2 + " " + choice3);
				}
			}
			
			outputRow(buf, item2.toString(), i, nextRow, row);
			
			double[] tmp = lastRow;
			lastRow = nextRow;
			nextRow = tmp;
		}
		double score = lastRow[m];		
		
		buf.append("\\end{tabular}\n");
		System.out.println(buf.toString());
	}
	
	public void outputRow(StringBuilder buf, String symbol, int index, double[] scores, char[] choice) { 
		buf.append(mc.replaceAll("ins", index+"") + " & " + cc1 + " \\prop{" + symbol + "} ");
		for (int i = 0; i < choice.length; ++i) { 
			switch (choice[i]) { 
			case 'l':
				buf.append(" & " + l);
				break;
			case 'd':
				buf.append(" & " + d);
				break;
			case 'u':
				buf.append(" & " + u);
				break;
			default:	
				buf.append(" & ");
			}
			
			int score = (int) scores[i];
			buf.append(" " + score);
		}
		buf.append(eol);
	}

	public static void main(String[] args) { 
		SignatureExample.init();
		
		SequenceAlignmentTable sat = new SequenceAlignmentTable();
		
		Map<String,List<Instance>> map = Utils.load("chpt1-", SequenceType.starts);
		List<Instance> list = map.get("chpt1-approach");

		Params p = new Params(list.get(2).sequence(), list.get(3).sequence());
		p.setMin(0, 0);
		p.setPenalty(-1, -1);
		p.setBonus(1, 1);
		p.subPenalty = -3;
		
		sat.latexTable(p);
	}
}
