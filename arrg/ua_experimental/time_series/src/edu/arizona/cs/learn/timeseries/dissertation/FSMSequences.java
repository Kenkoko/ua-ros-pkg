package edu.arizona.cs.learn.timeseries.dissertation;

import java.io.File;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.TreeSet;

import edu.arizona.cs.learn.algorithm.bpp.BPPFactory;
import edu.arizona.cs.learn.algorithm.markov.BPPNode;
import edu.arizona.cs.learn.algorithm.markov.FSMFactory;
import edu.arizona.cs.learn.timeseries.experiment.BitPatternGeneration;
import edu.arizona.cs.learn.timeseries.model.Instance;
import edu.arizona.cs.learn.timeseries.model.Interval;
import edu.arizona.cs.learn.timeseries.model.SequenceType;
import edu.arizona.cs.learn.timeseries.model.Signature;
import edu.arizona.cs.learn.timeseries.model.symbols.StringSymbol;
import edu.arizona.cs.learn.timeseries.model.symbols.Symbol;
import edu.arizona.cs.learn.timeseries.visualization.TableFactory;
import edu.arizona.cs.learn.util.Utils;
import edu.arizona.cs.learn.util.graph.Edge;
import edu.uci.ics.jung.graph.DirectedGraph;

public class FSMSequences {
	public static void main(String[] args) {
		posseFirstFour();
	}

	public static DirectedGraph<BPPNode, Edge> posseFirstFour() {
		SignatureExample.init();
		Map<Integer,List<Interval>> map = Utils.load(new File("data/input/chpt1-approach.lisp"));

		Set<String> propSet = new TreeSet<String>();
		for (List<Interval> list : map.values()) {
			for (Interval interval : list) {
				propSet.add(interval.name);
			}
		}
		List<String> props = new ArrayList<String>(propSet);

		List<List<Interval>> all = new ArrayList<List<Interval>>();
		for (Integer key : map.keySet()) { 
			if (key != 5)
				all.add(BPPFactory.compress(map.get(key), Interval.eff));
		}

		DirectedGraph<BPPNode,Edge> graph = FSMFactory.makeGraph(props, all, false);
		FSMFactory.toDot(graph, "data/graph/posse-1-4.dot", false);
		
		all = new ArrayList<List<Interval>>();
		all.add(BPPFactory.compress(map.get(5), Interval.eff));
		
		graph = FSMFactory.makeGraph(props, all, false);
		FSMFactory.toDot(graph, "data/graph/posse-5.dot", false);
		
		return (DirectedGraph<BPPNode, Edge>) graph;
	}
	
	
	public static DirectedGraph<BPPNode, Edge> regularMarkovSequences() {
		
		SignatureExample.init();
		Map<Integer,List<Interval>> map = Utils.load(new File("data/input/chpt1-approach.lisp"));

		Set<String> propSet = new TreeSet<String>();
		for (List<Interval> list : map.values()) {
			for (Interval interval : list) {
				propSet.add(interval.name);
			}
		}
		List<String> props = new ArrayList<String>(propSet);

		List<List<Interval>> all = new ArrayList<List<Interval>>();
		for (List<Interval> list : map.values()) {
			all.add(BPPFactory.compress(list, Interval.eff));
		}

		DirectedGraph<BPPNode,Edge> graph = FSMFactory.makeGraph(props, all, true);
		FSMFactory.toDot(graph, "data/graph/approach-all.dot", false);
		return (DirectedGraph<BPPNode, Edge>) graph;
	}

	/**
	 * Generate the FSM for the plain sequences.
	 * @param prefix
	 */
	public static void plainSequences(String prefix) {
		SignatureExample.init();

		Map<Integer,List<Interval>> map = Utils.load(new File("data/input/" + prefix + ".lisp"));
		Set<String> propSet = new TreeSet<String>();

		List<List<Interval>> all = new ArrayList<List<Interval>>();
		for (List<Interval> episode : map.values()) {
			all.add(BPPFactory.compress(episode, Interval.eff));

			for (Interval interval : episode) {
				propSet.add(interval.name);
			}
		}
		List<String> props = new ArrayList<String>(propSet);

		DirectedGraph<BPPNode,Edge> graph = FSMFactory.makeGraph(props, all, false);
		FSMFactory.toDot(graph, "data/graph/" + prefix + ".dot", false);
	}

	/**
	 * Pruned sequences
	 * @param prefix
	 * @param type
	 * @param min
	 */
	public static void testPrunedSequences(String prefix, SequenceType type, int min) {
		SignatureExample.init();

		Map<String,List<Instance>> map = Utils.load(prefix, type);
		List<Instance> examples = map.get(prefix);

		Set<String> propSet = new TreeSet<String>();
		Signature s = new Signature("approach");
		for (Instance instance : examples) {
			s.update(instance.sequence());
			for (Symbol obj : instance.sequence()) {
				StringSymbol ss = (StringSymbol) obj;
				for (Interval interval : ss.getIntervals())
					propSet.add(interval.name);
			}
		}
		s = s.prune(min);

		System.out.println(TableFactory.toLatex(s.table()));

		List<String> props = new ArrayList<String>(propSet);

		List<List<Interval>> all = BitPatternGeneration.getBPPs(prefix, s.table(), propSet);
		DirectedGraph<BPPNode,Edge> graph = FSMFactory.makeGraph(props, all, false);
		FSMFactory.toDot(graph, "data/graph/" + prefix + ".dot", false);
	}

	/**
	 * 
	 * @param key
	 * @param type
	 */
	public static void markovChainFromFile(String key, SequenceType type) {
		Signature s = Signature.fromXML("data/signatures/" + key + "-" + type
				+ ".xml");
		int min = (int) Math.floor(s.trainingSize() * 0.75D);
		s = s.prune(min);

		Set<String> propSet = new TreeSet<String>();
		for (Symbol obj : s.signature()) {
			StringSymbol ss = (StringSymbol) obj;
			propSet.addAll(ss.getProps());
		}
		List<String> props = new ArrayList<String>(propSet);
		List<List<Interval>> all = BitPatternGeneration.getBPPs(key, s.table(), propSet);

		DirectedGraph<BPPNode,Edge> graph = FSMFactory.makeGraph(props, all, false);
		FSMFactory.toDot(graph, "data/graph/" + key + "-" + type + ".dot",
				false);
//		DirectedGraph<BPPNode,Edge> minGraph = FSMFactory.minimize(graph, props);
//		FSMFactory.toDot(minGraph, "data/graph/" + key + "-" + type
//				+ "-min.dot", true);
	}
}