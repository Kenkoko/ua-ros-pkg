package edu.arizona.cs.learn.timeseries.dissertation;

import java.util.List;
import java.util.Map;

import edu.arizona.cs.learn.algorithm.alignment.GeneralAlignment;
import edu.arizona.cs.learn.algorithm.alignment.Params;
import edu.arizona.cs.learn.algorithm.alignment.Report;
import edu.arizona.cs.learn.timeseries.model.Instance;
import edu.arizona.cs.learn.timeseries.model.Interval;
import edu.arizona.cs.learn.timeseries.model.SequenceType;
import edu.arizona.cs.learn.timeseries.model.Signature;
import edu.arizona.cs.learn.timeseries.model.symbols.AllenRelation;
import edu.arizona.cs.learn.timeseries.model.symbols.Event;
import edu.arizona.cs.learn.timeseries.model.symbols.Symbol;
import edu.arizona.cs.learn.timeseries.visualization.TableFactory;
import edu.arizona.cs.learn.util.Utils;

public class SignatureExample {
	public static void init() {
		Utils.propMap.put("collision(agent,box)", "c(a,b)");
		Utils.propMap.put("distance-stable(agent,box2)", "ds(a,b2)");
		Utils.propMap.put("distance-decreasing(agent,box)", "dd(a,b)");
		Utils.propMap.put("forward(agent)", "f(a)");
		Utils.propMap.put("distance-decreasing(agent,box2)", "dd(a,b2)");
		Utils.propMap.put("speed-decreasing(agent)", "sd(a)");
		Utils.propMap.put("turn-left(agent)", "tl(a)");
		Utils.propMap.put("turn-right(agent)", "tr(a)");
		Utils.propMap.put("distance-increasing(agent,box2)", "di(a,b2)");
	}

	public static void main(String[] args) {
		init();
		makeMultipleSequenceAlignmentTable();
		if (true) return;
		
		AllenRelation.text = AllenRelation.fullText;
		Map<String,List<Instance>> map = Utils.load("chpt1", SequenceType.allen);
		List<Instance> list = map.get("chpt1-approach");
		Signature s = new Signature("chpt1-approach");
		for (int i = 1; i < list.size(); i++) {
			
			
			s.update(list.get(i).sequence());
		}
		s = s.prune(2);
		s.toXML("data/signatures/chpt1-approach.xml");
		latexPrintSignature("data/signatures/chpt1-approach.xml");
		
		Params params = new Params();
		params.seq1 = s.signature();
		params.seq2 = list.get(0).sequence();
		params.setMin(0, 0);
		params.setBonus(1.0D, 0.0D);
		params.setPenalty(-1.0D, 0.0D);
		Report report = GeneralAlignment.align(params);
		List<Symbol> results = GeneralAlignment.combineAlignments(report.results1, report.results2);
		for (int i = 0; i < report.results1.size(); ++i) { 
			StringBuffer buf = new StringBuffer();
			Symbol obj1 = report.results1.get(i);
			Symbol obj2 = report.results2.get(i);
			if (obj1 == null)  
				buf.append("$-$");
			else {
				AllenRelation ar = (AllenRelation) obj1;
				buf.append("\\ar{" + ar.prop1() + "}{" + ar.relation() + "}{" + ar.prop2() + "}");
			}
			buf.append(" & ");
			
			if (obj2 == null)
				buf.append("$-$");
			else {
				AllenRelation ar = (AllenRelation) obj2;
				buf.append("\\ar{" + ar.prop1() + "}{" + ar.relation() + "}{" + ar.prop2() + "}");
			}
			buf.append(" & ");
			
			Symbol result = results.get(i);
			AllenRelation ar = (AllenRelation) result;
			buf.append("\\ar{" + ar.prop1() + "}{" + ar.relation() + "}{" + ar.prop2() + "} & " + (int) result.weight());
			System.out.println(buf.toString() + " \\\\");
		}
		
//		printSignature("data/cross-validation/k6/fold-0/allen/ww2d-ball-prune.xml");
	}
	
	public static void latexPrintSignature(String xml) { 
		Signature s = Signature.fromXML(xml);
		for (Symbol obj : s.signature()) { 
			// Assumption is the WeightedObject symbol is an
			// Allen relation
			AllenRelation ar = (AllenRelation) obj;
			System.out.println("\\ar{" + ar.prop1() + "}{" + ar.relation() + "}{" + ar.prop2() + "} & " + ((int) obj.weight()) + "\\\\");
		}
	}

	public static void printSignature(String xml) {
		Signature s1 = Signature.fromXML(xml);
		int minSeen = (int) Math.round(s1.trainingSize() * 0.8D);
		s1 = s1.prune(minSeen);

		System.out.println("Signature: ww2d-ball pruned to " + minSeen);
		for (Symbol obj : s1.signature())
			System.out.println("\t" + obj.toString() + " - " + obj.weight());
	}

	/**
	 * Construct a set of signatures from the prefix...
	 *   -- example: makeSignature("ww3d-jump-over", SequenceType.allen, 3);
	 * @param prefix
	 * @param type
	 * @param min
	 */
	public static void makeSignature(String prefix, SequenceType type, int min) {
		Map<String,List<Instance>> map = Utils.load(prefix, type);

		for (String key : map.keySet()) {
			List<Instance> list = map.get(key);
			Signature s = new Signature(key);

			for (int i = 0; i < list.size(); i++) {
				s.update(list.get(i).sequence());
			}

			s = s.prune(min);
			s.toXML("data/signatures/" + key + "-" + type + ".xml");
		}
	}

	public static void makeSignatureUpdateTable() {
		Map<String,List<Instance>> map = Utils.load("chpt1-", SequenceType.starts);

		for (String key : map.keySet()) {
			List<Instance> list = map.get(key);
			Signature s = new Signature(key);

			for (int i = 0; i < list.size() - 1; i++) {
				s.update(((Instance) list.get(i)).sequence());
			}

			Params p = new Params(s.signature(), ((Instance) list.get(list
					.size() - 1)).sequence());
			p.setMin(0, 0);
			p.setBonus(1.0D, 0.0D);
			p.setPenalty(-1.0D, 0.0D);

			Report report = GeneralAlignment.align(p);

			List<Symbol> combined = GeneralAlignment.combineAlignments(report.results1, report.results2);
			for (int i = 0; i < report.results1.size(); i++) {
				Symbol left = report.results1.get(i);
				Symbol right = report.results2.get(i);

				String name = "null";
				if (left == null) {
					System.out.print("$-$ & ");
				} else {
					Event event = (Event) left;
					name = ((Interval) event.getIntervals().get(0)).name;
					System.out.print("\\prop{" + name + "} & ");
				}

				if (right == null) {
					System.out.print("$-$ & ");
				} else {
					Event event = (Event) right;
					name = ((Interval) event.getIntervals().get(0)).name;
					System.out.print("\\prop{" + name + "} & ");
				}

				Symbol obj = combined.get(i);
				System.out.println("\\prop{" + name + "} & "
						+ (int) obj.weight() + " \\\\");
			}
		}
	}

	public static void makeMultipleSequenceAlignmentTable() {
		Map<String,List<Instance>> map = Utils.load("chpt1-", SequenceType.allen);

		for (String key : map.keySet()) {
			List<Instance> list = map.get(key);
			Signature s = new Signature(key);

			for (int i = 0; i < list.size(); i++) {
				s.update(((Instance) list.get(i)).sequence());
			}
			s = s.prune(2);
			System.out.println(TableFactory.toLatex(s.table()));
		}
	}
}