package edu.arizona.cs.learn.timeseries.dissertation;

import java.util.List;
import java.util.Map;

import edu.arizona.cs.learn.algorithm.alignment.Params;
import edu.arizona.cs.learn.algorithm.alignment.Report;
import edu.arizona.cs.learn.algorithm.alignment.SequenceAlignment;
import edu.arizona.cs.learn.algorithm.alignment.model.Instance;
import edu.arizona.cs.learn.algorithm.alignment.model.WeightedObject;
import edu.arizona.cs.learn.timeseries.model.Event;
import edu.arizona.cs.learn.timeseries.model.Interval;
import edu.arizona.cs.learn.timeseries.model.Signature;
import edu.arizona.cs.learn.timeseries.visualization.TableFactory;
import edu.arizona.cs.learn.util.SequenceType;
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

		printSignature();
	}

	public static void printSignature() {
		Signature s1 = Signature.fromXML("data/cross-validation/k6/fold-0/allen/ww2d-ball-prune.xml");
		int minSeen = (int) Math.round(s1.trainingSize() * 0.8D);
		s1 = s1.prune(minSeen);

		System.out.println("Signature: ww2d-ball pruned to " + minSeen);
		for (WeightedObject obj : s1.signature())
			System.out.println("\t" + obj.key().getKey() + " - " + obj.weight());
	}

	public static void makeSignature(String prefix, SequenceType type, int min) {
		Map<String,List<Instance>> map = Utils.load(prefix, type);

		for (String key : map.keySet()) {
			List<Instance> list = map.get(key);
			Signature s = new Signature(key);

			for (int i = 0; i < list.size(); i++) {
				s.update(((Instance) list.get(i)).sequence());
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

			Report report = SequenceAlignment.align(p);

			List<WeightedObject> combined = SequenceAlignment.combineAlignments(report.results1, report.results2);
			for (int i = 0; i < report.results1.size(); i++) {
				WeightedObject left = (WeightedObject) report.results1.get(i);
				WeightedObject right = (WeightedObject) report.results2.get(i);

				String name = "null";
				if (left == null) {
					System.out.print("$-$ & ");
				} else {
					Event event = (Event) left.key();
					name = ((Interval) event.getIntervals().get(0)).name;
					System.out.print("\\prop{" + name + "} & ");
				}

				if (right == null) {
					System.out.print("$-$ & ");
				} else {
					Event event = (Event) right.key();
					name = ((Interval) event.getIntervals().get(0)).name;
					System.out.print("\\prop{" + name + "} & ");
				}

				WeightedObject obj = (WeightedObject) combined.get(i);
				System.out.println("\\prop{" + name + "} & "
						+ (int) obj.weight() + " \\\\");
			}
		}
	}

	public static void makeMultipleSequenceAlignmentTable() {
		Map<String,List<Instance>> map = Utils.load("chpt1-", SequenceType.starts);

		for (String key : map.keySet()) {
			List<Instance> list = map.get(key);
			Signature s = new Signature(key);

			for (int i = 0; i < list.size(); i++) {
				s.update(((Instance) list.get(i)).sequence());
			}
			System.out.println(TableFactory.toLatex(s.table()));
		}
	}
}