package edu.arizona.cs.learn.timeseries.evaluation;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;

import edu.arizona.cs.learn.algorithm.alignment.Similarity;
import edu.arizona.cs.learn.algorithm.markov.BPPNode;
import edu.arizona.cs.learn.algorithm.markov.FSMConverter;
import edu.arizona.cs.learn.algorithm.markov.FSMFactory;
import edu.arizona.cs.learn.algorithm.markov.FSMRecognizer;
import edu.arizona.cs.learn.timeseries.model.Instance;
import edu.arizona.cs.learn.timeseries.model.Signature;
import edu.arizona.cs.learn.timeseries.model.SignatureCallable;
import edu.arizona.cs.learn.timeseries.recognizer.Recognizer;
import edu.arizona.cs.learn.util.Utils;
import edu.arizona.cs.learn.util.graph.Edge;
import edu.uci.ics.jung.graph.DirectedGraph;

public class BatchSignatures {

	private Map<String,Signature> _signatures;
	private Map<String,FSMRecognizer> _recognizers;
	private Map<String,Long> _timing;
	
	private Map<String,List<Instance>> _training;
	
	private boolean _prune;
	private int _min;
	
	private Similarity _sim;
		
	public BatchSignatures(Map<String,List<Instance>> training) {
		this(training, false, 0);
	}
	
	public BatchSignatures(Map<String,List<Instance>> training, boolean prune, int min) { 
		this(training, false, 0, Similarity.strings);
	}
	
	public BatchSignatures(Map<String,List<Instance>> training, boolean prune, int min, Similarity sim) { 
		_training = training;
		_prune = prune;
		_min = min;
		_sim = sim;
		
		_signatures = null;
	}
	
	/**
	 * return the learned signature
	 * @return
	 */
	public Map<String,Signature> signatures() { 
		return _signatures;
	}
	
	public Map<String,FSMRecognizer> recognizers() { 
		return _recognizers;
	}
	
	/**
	 * return the timing information from the learned signatures.
	 * @return
	 */
	public Map<String,Long> timing() { 
		return _timing;
	}
	
	public void makeSignatures() { 
		ExecutorService execute = Executors.newFixedThreadPool(Utils.numThreads);

		List<Future<SignatureCallable>> futureList = new ArrayList<Future<SignatureCallable>>();
		for (String key : _training.keySet()) {
			SignatureCallable sc = new SignatureCallable(key, _prune, _min, _sim, _training.get(key));
			futureList.add(execute.submit(sc));
		}
		
		_signatures = new HashMap<String,Signature>();
		_timing = new HashMap<String,Long>();
		for (Future<SignatureCallable> results : futureList) {
			try {
				SignatureCallable sc = results.get();
				Signature signature = sc.signature();

				_timing.put(signature.key(), sc.duration());
				_signatures.put(signature.key(), signature);
			} catch (Exception e) { 
				e.printStackTrace();
			}
		}
		execute.shutdown();
	}
	
	public Map<String,FSMRecognizer> makeRecognizers(Recognizer r, int minPct) { 
		if (_signatures == null)
			makeSignatures();
		
		_recognizers = new HashMap<String,FSMRecognizer>();
		for (String key : _signatures.keySet()) { 
			FSMRecognizer fsm = r.build(key, _signatures.get(key), minPct, false);
			_recognizers.put(key, fsm);
		}
		return _recognizers;
	}
	
	public void writeSignatures(String directory) { 
		for (Signature s : _signatures.values()) 
			s.toXML(directory + s.key() + ".xml");
	}

	public void writeSignatures(String directory, int minPct) { 
		double pct = minPct / 100.0D;
		for (Signature s : _signatures.values()) {
			int minSeen = (int) Math.round(s.trainingSize() * pct);
			Signature tmp = s.prune(minSeen);
			tmp.toXML(directory + tmp.key() + ".xml");
		}
	}
	
	public void writeRecognizers(String directory) { 
		for (FSMRecognizer fsm : _recognizers.values()) {
			FSMFactory.toDot(fsm.getGraph(), directory + fsm.key() + ".dot");

			DirectedGraph<BPPNode, Edge> dfa = FSMConverter.convertNFAtoDFA(fsm.getGraph());
			FSMRecognizer mr = new FSMRecognizer(fsm.key(), dfa);
			FSMFactory.toDot(mr.getGraph(), directory + "opt-" + fsm.key() + ".dot");
		}
	}
}
