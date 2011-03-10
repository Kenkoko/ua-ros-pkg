package edu.arizona.verbs.verb.vfsm.learner;

import java.util.HashMap;
import java.util.List;
import java.util.Set;
import java.util.TreeSet;

import edu.arizona.cs.learn.algorithm.markov.BPPNode;
import edu.arizona.cs.learn.algorithm.markov.FSMRecognizer;
import edu.arizona.cs.learn.timeseries.model.Signature;
import edu.arizona.cs.learn.timeseries.model.symbols.Symbol;
import edu.arizona.cs.learn.timeseries.recognizer.Recognizer;
import edu.arizona.cs.learn.util.graph.Edge;
import edu.arizona.verbs.fsm.VerbFSM;
import edu.arizona.verbs.mdp.StateConverter;
import edu.arizona.verbs.shared.OOMDPState;
import edu.uci.ics.jung.graph.DirectedGraph;

public class SignatureLearner implements VFSMLearner {

	private String key_ = "learning"; // dummy value
	private Signature signature_ = new Signature(key_);
	private double percentage_;
	
	private VerbFSM vfsm_ = new VerbFSM();
	
	public SignatureLearner(double percentage) {
		percentage_ = percentage;
	}
	
	@Override
	public void addTrace(List<OOMDPState> trace) {
		signature_.update(StateConverter.convertTrace(trace, new HashMap<String, String>()));
		updateVFSM();
	}

	@Override
	public void forgetTraces() {
		signature_ = new Signature(key_);
		vfsm_ = new VerbFSM();
	}
	
	@Override
	public VerbFSM getFSM() {
		return vfsm_;
	}
	
	private void updateVFSM() {
//		int trainingSize = signature_.trainingSize();
//		int min = (int) Math.floor(trainingSize * percentage_);
		
		FSMRecognizer recognizer = Recognizer.cave.build(key_, signature_, (int) Math.round(percentage_ * 100), false);

		vfsm_ = new VerbFSM(recognizer);
	}
}
