package edu.arizona.cs.learn.timeseries.dissertation;

import java.io.File;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

import org.apache.commons.math.stat.descriptive.SummaryStatistics;

import edu.arizona.cs.learn.timeseries.model.Interval;
import edu.arizona.cs.learn.timeseries.model.SequenceType;
import edu.arizona.cs.learn.util.Utils;

public class DatasetsInfo {
	public static void main(String[] args) {
		//		makeLatexTable3("vowel");

		//		AllenRelation.MARGIN = 1;
		//		Utils.WINDOW = 1;
		getStats("data/input/", "ww3d");
		getStats("data/input/", "global-ww2d");
		getStats("/Users/wkerr/Sync/data/handwriting/wes/lisp/original/", "wes-pen");
	}

	public static void getStats(String dir, String prefix) { 
		SummaryStatistics numSteps = new SummaryStatistics();
		SummaryStatistics intervalSize = new SummaryStatistics();
		SummaryStatistics sequenceSize = new SummaryStatistics();
		SummaryStatistics propSize = new SummaryStatistics();

		for (File f : new File(dir).listFiles())
			if ((f.getName().startsWith(prefix))
					&& (f.getName().endsWith("lisp"))) {
				String name = f.getName();
				String className = name.substring(0, name.indexOf(".lisp"));
				Map<Integer,List<Interval>> map = Utils.load(f);

				System.out.println("[" + prefix + "] " + className + " " + map.size());

				for (List<Interval> list : map.values()) {
					int minStart = Integer.MAX_VALUE;
					int maxEnd = 0;

					Set<String> props = new HashSet<String>();
					for (Interval interval : list) {
						minStart = Math.min(minStart, interval.start);
						maxEnd = Math.max(maxEnd, interval.end);
						props.add(interval.name);
					}

					propSize.addValue(props.size());
					numSteps.addValue(maxEnd - minStart);
					intervalSize.addValue(list.size());
					sequenceSize.addValue(SequenceType.allen.getSequenceSize(list));
					System.gc();
				}
			}
		System.out.println(" Steps: " + numSteps.getMean());
		System.out.println(" Propositions: " + propSize.getMean());
		System.out.println(" Intervals: " + intervalSize.getMean());
		System.out.println(" Sequence: " + sequenceSize.getMean());
	}

	public static void datasetsInfo() {
		String[] prefixes = { "ww3d", "wes", "nicole", "derek", "vowel",
				"auslan", "wafer", "ecg", "ww2d" };

		List<SequenceType> types = new ArrayList<SequenceType>();
		types.add(SequenceType.allen);
		types.add(SequenceType.bpp);

		System.out
		.println("dataset,activity,num-examples,mean-steps,mean-num-intervals,mean-allen,mean-bpp");
		for (String prefix : prefixes) { 
			int numClasses = 0;
			for (File f : new File("data/input/").listFiles())
				if ((f.getName().startsWith(prefix))
						&& (f.getName().endsWith("lisp"))) {
					String name = f.getName();
					String className = name.substring(0, name.indexOf(".lisp"));
					Map<Integer,List<Interval>> map = Utils.load(f);

					StringBuffer buf = new StringBuffer(prefix + ","
							+ className + "," + map.size() + ",");

					SummaryStatistics numSteps = new SummaryStatistics();
					SummaryStatistics intervalSize = new SummaryStatistics();

					Map<SequenceType,SummaryStatistics> sequenceMap = new HashMap<SequenceType,SummaryStatistics>();
					for (SequenceType type : types) {
						sequenceMap.put(type, new SummaryStatistics());
					}
					for (List<Interval> list : map.values()) {
						int minStart = Integer.MAX_VALUE;
						int maxEnd = 0;

						for (Interval interval : list) {
							minStart = Math.min(minStart, interval.start);
							maxEnd = Math.max(maxEnd, interval.end);
						}
						numSteps.addValue(maxEnd - minStart);
						intervalSize.addValue(list.size());

						for (SequenceType type : types) {
							sequenceMap.get(type).addValue(type.getSequenceSize(list));
						}

						System.gc();
					}

					buf.append(numSteps.getMean() + ","
							+ intervalSize.getMean() + ",");

					for (SequenceType type : types) {
						buf.append(((SummaryStatistics) sequenceMap.get(type))
								.getMean() + ",");
					}

					numClasses++;
					System.out.println(buf.toString());
				}
		}
	}

	public static void makeLatexTable(String prefix) {
		int numClasses = 0;

		List<String> activities = new ArrayList<String>();
		List<Map<Integer,List<Interval>>> episodes = new ArrayList<Map<Integer,List<Interval>>>();

		List<SequenceType> types = new ArrayList<SequenceType>();
		types.add(SequenceType.allen);
		types.add(SequenceType.bpp);

		for (File f : new File("data/input/").listFiles()) {
			if ((f.getName().startsWith(prefix))
					&& (f.getName().endsWith("lisp"))) {
				String name = f.getName();
				String className = name.substring(0, name.indexOf(".lisp"));
				Map<Integer,List<Interval>> map = Utils.load(f);

				activities.add(className.substring(prefix.length() + 1));
				episodes.add(map);
			}

		}

		System.out.println("\\begin{table}");
		System.out.println("\\begin{footnotesize}");
		System.out.print("\\begin{tabular}{|c");
		for (int i = 0; i < activities.size(); i++)
			System.out.print("|c");
		System.out.println("|}");
		System.out.println("\\hline");

		StringBuffer buf = new StringBuffer("  ");
		for (String activity : activities) {
			buf.append(" & {\\it " + activity + "}");
		}
		System.out.println(buf + " \\\\ \\hline");

		buf = new StringBuffer("Num. Examples ");
		for (Map<Integer,List<Interval>> map : episodes)
			buf.append(" & " + map.size());
		System.out.println(buf + " \\\\ \\hline");

		StringBuffer timeBuf = new StringBuffer("Avg. Time  ");
		StringBuffer fluentsBuf = new StringBuffer("Avg. Num. Fluents ");

		StringBuffer sEvent = new StringBuffer("Avg. Event (starts) ");
		StringBuffer eEvent = new StringBuffer("Avg. Event (ends) ");
		StringBuffer bEvent = new StringBuffer("Avg. Event (both) ");

		StringBuffer allen = new StringBuffer("Avg. Allen ");
		StringBuffer cba = new StringBuffer("Avg. CBA ");

		for (int i = 0; i < activities.size(); i++) {
			Map<Integer,List<Interval>> map = episodes.get(i);

			SummaryStatistics numSteps = new SummaryStatistics();
			SummaryStatistics intervalSize = new SummaryStatistics();

			Map<SequenceType,SummaryStatistics> sequenceMap = new HashMap<SequenceType,SummaryStatistics>();
			for (SequenceType type : types) {
				sequenceMap.put(type, new SummaryStatistics());
			}
			for (List<Interval> list : map.values()) {
				int minStart = Integer.MAX_VALUE;
				int maxEnd = 0;

				for (Interval interval : list) {
					minStart = Math.min(minStart, interval.start);
					maxEnd = Math.max(maxEnd, interval.end);
				}
				numSteps.addValue(maxEnd - minStart);
				intervalSize.addValue(list.size());

				for (SequenceType type : types) {
					((SummaryStatistics) sequenceMap.get(type)).addValue(type
							.getSequenceSize(list));
				}

				System.gc();
			}

			timeBuf.append(" & " + Utils.nf.format(numSteps.getMean()));
			fluentsBuf.append(" & " + Utils.nf.format(intervalSize.getMean()));

			sEvent.append(" & " + Utils.nf.format(intervalSize.getMean()));
			eEvent.append(" & " + Utils.nf.format(intervalSize.getMean()));
			bEvent.append(" & "
					+ Utils.nf.format(2.0D * intervalSize.getMean()));

			allen.append(" & "
					+ Utils.nf.format(((SummaryStatistics) sequenceMap
							.get(SequenceType.allen)).getMean()));
			cba.append(" & "
					+ Utils.nf.format(((SummaryStatistics) sequenceMap
							.get(SequenceType.bpp)).getMean()));

			numClasses++;
		}

		System.out.println(timeBuf + " \\\\ \\hline");
		System.out.println(fluentsBuf + " \\\\ \\hline");
		System.out.println(sEvent + " \\\\ \\hline");
		System.out.println(eEvent + " \\\\ \\hline");
		System.out.println(bEvent + " \\\\ \\hline");
		System.out.println(allen + " \\\\ \\hline");
		System.out.println(cba + " \\\\ \\hline");

		System.out.println("\\end{tabular}");
		System.out.println("\\end{footnotesize}");
		System.out.println("\\end{table}");
	}

	public static void makeLatexTable2(String prefix) {
		int numClasses = 0;

		List<String> activities = new ArrayList<String>();
		List<Map<Integer,List<Interval>>> episodes = new ArrayList<Map<Integer,List<Interval>>>();

		List<SequenceType> types = new ArrayList<SequenceType>();
		types.add(SequenceType.allen);
		types.add(SequenceType.bpp);

		for (File f : new File("data/input/").listFiles()) {
			if ((f.getName().startsWith(prefix))
					&& (f.getName().endsWith("lisp"))) {
				String name = f.getName();
				String className = name.substring(0, name.indexOf(".lisp"));
				Map<Integer,List<Interval>> map = Utils.load(f);

				activities.add(className.substring(prefix.length() + 1));
				episodes.add(map);
			}

		}

		System.out.println("\\begin{table}");
		System.out.println("\\centering");
		System.out.println("\\begin{footnotesize}");
		System.out.print("\\begin{tabular}{|c|c|c|c|c|c|}");
		System.out.println("\\hline");

		System.out
		.println("  & Num Examples & Time & Num Fluents & Allen & CBA \\\\ \\hline");
		for (int i = 0; i < activities.size(); i++) {
			StringBuffer buf = new StringBuffer("{\\it "
					+ (String) activities.get(i) + "} ");

			Map<Integer,List<Interval>> map = episodes.get(i);
			buf.append(" & " + map.size());

			SummaryStatistics numSteps = new SummaryStatistics();
			SummaryStatistics intervalSize = new SummaryStatistics();

			Map<SequenceType,SummaryStatistics> sequenceMap = new HashMap<SequenceType,SummaryStatistics>();
			for (SequenceType type : types) {
				sequenceMap.put(type, new SummaryStatistics());
			}

			for (List<Interval> list : map.values()) {
				int minStart = Integer.MAX_VALUE;
				int maxEnd = 0;

				for (Interval interval : list) {
					minStart = Math.min(minStart, interval.start);
					maxEnd = Math.max(maxEnd, interval.end);
				}
				numSteps.addValue(maxEnd - minStart);
				intervalSize.addValue(list.size());

				for (SequenceType type : types) {
					((SummaryStatistics) sequenceMap.get(type)).addValue(type
							.getSequenceSize(list));
				}

				System.gc();
			}

			buf.append(" & " + Utils.nf.format(numSteps.getMean()));
			buf.append(" & " + Utils.nf.format(intervalSize.getMean()));

			buf.append(" & "
					+ Utils.nf.format(((SummaryStatistics) sequenceMap
							.get(SequenceType.allen)).getMean()));
			buf.append(" & "
					+ Utils.nf.format(((SummaryStatistics) sequenceMap
							.get(SequenceType.bpp)).getMean()));

			System.out.println(buf + " \\\\ \\hline");
			numClasses++;
		}
		System.out.println("\\end{tabular}");
		System.out.println("\\end{footnotesize}");
		System.out.println("\\end{table}");
	}

	public static void makeLatexTable3(String prefix) {
		int numClasses = 0;

		List<String> activities = new ArrayList<String>();
		List<Map<Integer,List<Interval>>> episodes = new ArrayList<Map<Integer,List<Interval>>>();

		List<SequenceType> types = new ArrayList<SequenceType>();
		types.add(SequenceType.allen);
		types.add(SequenceType.bpp);

		for (File f : new File("data/input/").listFiles()) {
			if ((f.getName().startsWith(prefix))
					&& (f.getName().endsWith("lisp"))) {
				String name = f.getName();
				String className = name.substring(0, name.indexOf(".lisp"));
				Map<Integer,List<Interval>> map = Utils.load(f);

				activities.add(className.substring(prefix.length() + 1));
				episodes.add(map);
			}

		}

		System.out.println("\\begin{table}");
		System.out.println("\\begin{footnotesize}");
		System.out.print("\\begin{tabular}{|c|c|}");
		System.out.println("\\hline");
		System.out.println("    & Auslan \\\\ \\hline");
		System.out.println("Num. Classes & " + episodes.size());

		SummaryStatistics mExamples = new SummaryStatistics();
		SummaryStatistics mTime = new SummaryStatistics();
		SummaryStatistics mFluents = new SummaryStatistics();

		SummaryStatistics mAllen = new SummaryStatistics();
		SummaryStatistics mCBA = new SummaryStatistics();

		for (int i = 0; i < episodes.size(); i++) {
			Map<Integer,List<Interval>> map = episodes.get(i);

			mExamples.addValue(map.size());
			for (List<Interval> list : map.values()) {
				int minStart = Integer.MAX_VALUE;
				int maxEnd = 0;

				for (Interval interval : list) {
					minStart = Math.min(minStart, interval.start);
					maxEnd = Math.max(maxEnd, interval.end);
				}
				mTime.addValue(maxEnd - minStart);
				mFluents.addValue(list.size());

				mAllen.addValue(SequenceType.allen.getSequenceSize(list));
				mCBA.addValue(SequenceType.bpp.getSequenceSize(list));
			}
		}

		System.out.println("Num. Examples & "
				+ Utils.nf.format(mExamples.getMean()) + " \\\\ \\hline");
		System.out.println("Time          & "
				+ Utils.nf.format(mTime.getMean()) + " \\\\ \\hline");
		System.out.println("Fluents       & "
				+ Utils.nf.format(mFluents.getMean()) + " \\\\ \\hline");
		System.out.println("Allen         & "
				+ Utils.nf.format(mAllen.getMean()) + " \\\\ \\hline");
		System.out.println("CBA           & " + Utils.nf.format(mCBA.getMean())
				+ " \\\\ \\hline");

		System.out.println("\\end{tabular}");
		System.out.println("\\end{footnotesize}");
		System.out.println("\\end{table}");
	}
}