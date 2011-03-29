package edu.arizona.cs.learn.timeseries.recognizer;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.TreeSet;

import edu.arizona.cs.learn.algorithm.bpp.BPPFactory;
import edu.arizona.cs.learn.algorithm.markov.BPPNode;
import edu.arizona.cs.learn.algorithm.markov.FSMFactory;
import edu.arizona.cs.learn.algorithm.markov.FSMRecognizer;
import edu.arizona.cs.learn.timeseries.experiment.BitPatternGeneration;
import edu.arizona.cs.learn.timeseries.model.Interval;
import edu.arizona.cs.learn.timeseries.model.Signature;
import edu.arizona.cs.learn.timeseries.model.symbols.StringSymbol;
import edu.arizona.cs.learn.timeseries.model.symbols.Symbol;
import edu.arizona.cs.learn.util.graph.Edge;
import edu.uci.ics.jung.graph.DirectedGraph;

public enum Recognizer {
	cave {
		@Override
		public FSMRecognizer build(
				String key, String signatureFile,
				Map<Integer, List<Interval>> training, List<Integer> test,
				int minPct, boolean onlyStart) {
			Signature s = Signature.fromXML(signatureFile);
			return build(key, s, minPct, onlyStart);
		}

		@Override
		public FSMRecognizer build(String key, Signature s, int minPct, boolean onlyStart) {
			double pct = minPct / 100.0D;

			// Daniel: I changed this to a floor to avoid getting empty FSMs
			int minSeen = (int) Math.floor(s.trainingSize() * pct);
			s = s.prune(minSeen);

			// Assumption is that you will only build FSMRecognizer on 
			// StringSymbols....for now
			// TODO: extend to work with ComplexSymbols (but all binary)
			Set<String> propSet = new TreeSet<String>();
			for (Symbol obj : s.signature()) {
				StringSymbol ss = (StringSymbol) obj;
				propSet.addAll(ss.getProps());
			}
			List<String> props = new ArrayList<String>(propSet);
			List<List<Interval>> all = BitPatternGeneration.getBPPs(null, s.table(), propSet);

			DirectedGraph<BPPNode, Edge> graph = FSMFactory.makeGraph(props, all, onlyStart);
			return new FSMRecognizer(key, graph);
		}
	},
	regular {
		@Override
		public FSMRecognizer build(
				String key, String signatureFile,
				Map<Integer, List<Interval>> training, List<Integer> test,
				int minPct, boolean onlyStart) {

			List<List<Interval>> bpps = new ArrayList<List<Interval>>();
			Set<String> propSet = new TreeSet<String>();
			for (Integer id : training.keySet()) {
				if (test.indexOf(id) != -1) {
					continue;
				}
				
				bpps.add(BPPFactory.compress(training.get(id), Interval.eff));
				for (Interval interval : training.get(id)) {
					propSet.add(interval.name);
				}
			}

			List<String> props = new ArrayList<String>(propSet);
			DirectedGraph<BPPNode, Edge> graph = FSMFactory.makeGraph(props, bpps, onlyStart);

			FSMFactory.toDot(graph, "data/graph/tmp-" + key + ".dot");

			return new FSMRecognizer(key, graph);
		}

		@Override
		public FSMRecognizer build(String key, Signature s, int minPct, boolean onlyStart) {
			throw new RuntimeException("Not yet implemented!!");
		}

	};
	
	public abstract FSMRecognizer build(String key, Signature s, int minPct, boolean onlyStart);

	public abstract FSMRecognizer build(String key, String signatureFile,
			Map<Integer, List<Interval>> training, List<Integer> test,
			int minPct, boolean onlyStart);
}