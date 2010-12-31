package edu.arizona.cs.learn.timeseries.dissertation;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.TreeSet;

import edu.arizona.cs.learn.algorithm.markov.BPPNode;
import edu.arizona.cs.learn.algorithm.markov.FSMFactory;
import edu.arizona.cs.learn.algorithm.markov.FSMRecognizer;
import edu.arizona.cs.learn.timeseries.Experiments;
import edu.arizona.cs.learn.timeseries.experiment.BitPatternGeneration;
import edu.arizona.cs.learn.timeseries.model.Interval;
import edu.arizona.cs.learn.timeseries.model.SequenceType;
import edu.arizona.cs.learn.timeseries.model.Signature;
import edu.arizona.cs.learn.timeseries.model.symbols.StringSymbol;
import edu.arizona.cs.learn.timeseries.model.symbols.Symbol;
import edu.arizona.cs.learn.timeseries.recognizer.Recognizer;
import edu.arizona.cs.learn.timeseries.recognizer.RecognizerStatistics;
import edu.arizona.cs.learn.util.Utils;
import edu.arizona.cs.learn.util.graph.Edge;
import edu.uci.ics.jung.graph.DirectedGraph;

public class RecognitionExperiment {
	public static void main(String[] args) {
//		Signature s = new Signature("jump-over");
//		Map<Integer,List<Interval>> episodes = Utils.load(new File("data/input/ww3d-jump-over.lisp"));
//		
//		int i = 1;
//		for (Integer eid : episodes.keySet()) { 
//			
//			s.update(SequenceType.allen.getSequence(episodes.get(eid)));
//			if (i % 10 == 0){
//				s = s.prune(3);
//				System.out.println("Pruning");
//			}
//			++i;
//		}
//		
//		s.toXML("data/signatures/jump-over.xml");
//		s = s.prune(25);
//		Set<String> propSet = new TreeSet<String>();
//		for (WeightedObject obj : s.signature()) {
//			propSet.addAll(obj.key().getProps());
//		}
//		List<String> props = new ArrayList<String>(propSet);
//		List<List<Interval>> all = BitPatternGeneration.getBPPs(null, s.table(), propSet);
//
//		DirectedGraph<BPPNode, Edge> graph = FSMFactory.makeGraph(props, all, false);
//		FSMFactory.toDot(graph, "data/graph/tmp.dot");
		
		
		
		List<String> classes = new ArrayList<String>();
		classes.add("jump-on");
		classes.add("jump-over");
		classes.add("left");
		classes.add("right");
		classes.add("push");
		classes.add("approach");

		int[] folds = { 6 };
		int[] pcts = { 80 };
		SequenceType type = SequenceType.allen;
		String prefix = "ww3d";
		try {
			BufferedWriter out = new BufferedWriter(new FileWriter("data/recognizer-" + prefix + "-" + type + "-all.csv"));
			out.write("className,type,fold,recognizer,minPct,precision,recall,f\n");
			for (int i : pcts) {
				for (int fold : folds) {
//					Experiments.selectCrossValidation("ww3d", fold);
					Experiments cv = new Experiments(fold);

					Map<String, RecognizerStatistics> map = 
						cv.recognition(prefix, Recognizer.cave, type, i, true, false);
					for (String className : map.keySet()) {
						RecognizerStatistics rs = map.get(className);
						out.write(className + "," + type + "," + fold
								+ ",cave," + i + "," + rs.precision() + ","
								+ rs.recall() + "," + rs.fscore() + "\n");
					}
				}
			}
			out.close();
		} catch (Exception e) {
			e.printStackTrace();
		}
	}

	/**
	 * 
	 * @param dataset
	 * @param classes
	 * @param type
	 * @return
	 */
	public static List<FSMRecognizer> makeRecognizers(String dataset,
			List<String> classes, SequenceType type) {
		String path = "data/signatures/";

		List<FSMRecognizer> results = new ArrayList<FSMRecognizer>();
		for (String className : classes) {
			String file = path + dataset + "-" + className + "-" + type + ".xml";
			Signature s = Signature.fromXML(file);

			Set<String> propSet = new TreeSet<String>();
			for (Symbol obj : s.signature()) {
				StringSymbol ss = (StringSymbol) obj;
				propSet.addAll(ss.getProps());
			}
			List<String> props = new ArrayList<String>(propSet);
			List<List<Interval>> all = BitPatternGeneration.getBPPs(null, s.table(), propSet);

			DirectedGraph<BPPNode, Edge> graph = FSMFactory.makeGraph(props, all, false);
			FSMFactory.toDot(graph, "data/graph/tmp.dot");
			FSMRecognizer mr = new FSMRecognizer(className, graph);
			results.add(mr);
		}
		return results;
	}

	/**
	 * 
	 * @param input
	 * @param recognizers
	 */
	public static void recognition(String input, List<FSMRecognizer> recognizers) {
		Map<Integer,List<Interval>> map = Utils.load(new File(input));

		for (Integer id : map.keySet()) {
			System.out.println("Episode ID: " + id);
			List<Interval> intervals = map.get(id);

			int start = Integer.MAX_VALUE;
			int end = 0;
			for (Interval interval : intervals) {
				start = Math.min(start, interval.start);
				end = Math.max(end, interval.end);
			}

			for (int i = start; i < end; i++) {
				Set<String> props = new HashSet<String>();
				for (Interval interval : intervals) {
					if (interval.on(i)) {
						props.add(interval.name);
					}
				}

				boolean recognized = false;
				for (FSMRecognizer mr : recognizers) {
					recognized = (recognized) || (mr.update(props));
				}

				if (recognized)
					System.out.println("TIME: " + i);
			}
		}
	}
}