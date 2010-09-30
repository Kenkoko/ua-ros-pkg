package edu.arizona.verbs;

import java.io.File;
import java.util.ArrayList;
import java.util.Collection;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.TreeSet;
import java.util.Vector;

import org.apache.log4j.Logger;

import edu.arizona.cs.learn.algorithm.alignment.Params;
import edu.arizona.cs.learn.algorithm.alignment.SequenceAlignment;
import edu.arizona.cs.learn.algorithm.alignment.model.Instance;
import edu.arizona.cs.learn.algorithm.alignment.model.WeightedObject;
import edu.arizona.cs.learn.algorithm.markov.BPPNode;
import edu.arizona.cs.learn.algorithm.markov.FSMFactory;
import edu.arizona.cs.learn.algorithm.markov.FSMRecognizer;
import edu.arizona.cs.learn.timeseries.experiment.BitPatternGeneration;
import edu.arizona.cs.learn.timeseries.model.Interval;
import edu.arizona.cs.learn.timeseries.model.Signature;
import edu.arizona.cs.learn.util.graph.Edge;
import edu.uci.ics.jung.graph.DirectedGraph;

public class Verb {
	private static Logger logger = Logger.getLogger(Verb.class);
	
	private String lexicalForm_;
	private Signature signature_ = null;
	private DirectedGraph<BPPNode,Edge> fsm_ = null;
	private FSMRecognizer recognizer_ = null;
	
	public Verb(String word) {
		lexicalForm_ = word;
		makeVerbFolder();
	}
	
	public Verb(String word, Signature signature) {
		lexicalForm_ = word;
		makeVerbFolder();
		signature_ = signature;
		makeRecognizer();
	}
	
	public void generalizeInstances(List<Instance> instances) {
		signature_ = new Signature(lexicalForm_);
		for (Instance instance : instances) {
			signature_.update(instance.sequence());
		}

		// Let's only print the relations that are in all the episodes
		Signature consensus = signature_.prune(instances.size() - 1);
		
		for (WeightedObject obj : consensus.signature()) {
			logger.debug("\t" + obj.key().getKey() + " - " + obj.weight());
		}

		// This is not strictly necessary, but useful in case of crashes, etc.
		String filename = getVerbFolder() + "signature.xml"; 
		signature_.toXML(filename);
		logger.debug("Signature saved to file: " + filename);
		
		makeRecognizer();
	}
	
	private void makeRecognizer() {
		if (!hasSignature()) {
			logger.error("WTF are you doing? There is no signature for " + lexicalForm_);
			return;
		}
		
		Set<String> propSet = new TreeSet<String>();
		for (WeightedObject obj : signature_.signature()) {
			propSet.addAll(obj.key().getProps());
		}
		List<String> props = new ArrayList<String>(propSet);
		List<List<Interval>> all = BitPatternGeneration.getBPPs(lexicalForm_, signature_.table(), propSet);

		fsm_ = FSMFactory.makeGraph(props, all, false);
		FSMFactory.toDot(fsm_, getVerbFolder() + "raw_fsm.dot");
		StateMachineUtils.convertToDFA(fsm_);
		FSMFactory.toDot(fsm_, getVerbFolder() + "fsm.dot");
		
		recognizer_ = new FSMRecognizer(lexicalForm_, fsm_);
	}
	
	public void addConstraint(Collection<String> bannedProps) {
		StringBuffer fluentStates = new StringBuffer();
		for (@SuppressWarnings("unused") String prop : bannedProps) { 
			fluentStates.append("1"); 
		}
		BPPNode deadState = new BPPNode(new Vector<String>(bannedProps), fluentStates, recognizer_.getStartState());
		deadState.color("red");
		deadState.fontColor("white");
		
		fsm_.addVertex(deadState);
		for (BPPNode state : fsm_.getVertices()) {
			if (fsm_.getSuccessorCount(state) > 0) { // Goal states are unaffected
				Edge e = new Edge(new HashSet<String>(bannedProps));
				fsm_.addEdge(e, state, deadState);
			}
		}
		
		// TODO: Need to mark these states as bad so that the reward function can work right, otherwise they look like goals!
		// Also make them look different in the FSM
		
		FSMFactory.toDot(fsm_, getVerbFolder() + "fsm-constrained.dot");
		recognizer_ = new FSMRecognizer(lexicalForm_, fsm_); // Have to update the recognizer since the FSM changed
	}
	
	public double testInstance(Instance instance, int min) {
		logger.debug("Computing Alignment Score...");
		
		Params params = new Params();
		params.setMin(min, 0); // Rule of thumb is half the number of instances in the signature, maybe should set that if -1
		params.setBonus(1, 0);
		params.setPenalty(-1, 0);
		params.seq1 = signature_.signature();
		params.seq2 = instance.sequence();
		
		return SequenceAlignment.distance(params);
	}
	
	public double updateFSM(Set<String> activeProps) {
		logger.debug("Updating FSM State...");
		
		List<BPPNode> oldActives = recognizer_.getActiveExceptStart();
		
		boolean updateResult = recognizer_.update(activeProps);
		
//		for (String prop : activeProps) {
//			logger.debug("\t[" + prop + "]");
//		}
		
		if (updateResult) {
			logger.debug("Goal State Reached! FSM accepted sequence.");
			return 1.0; 
		} 
		
		List<BPPNode> newActives = recognizer_.getActiveExceptStart();
		if (newActives.isEmpty()) {
			logger.debug("In start state");
			return -2.0;
			
		} else {
			logger.debug("Active States (except start state):");
			
			for (BPPNode newState : newActives) {
				logger.debug("\t" + newState.label());
			}
			
			if (oldActives.isEmpty()) { // Must have moved out of start state
				return 0.5;
			}
			
			for (BPPNode newState : newActives) {
				for (BPPNode oldState : oldActives) { // TODO: If any new state is a successor to any old state, return 0.5
					if (fsm_.isSuccessor(newState, oldState)) {
						return 0.5; // Progress
					} else if (fsm_.isPredecessor(newState, oldState)) {
						return -1.0; // Moving backwards
					} else { 
						return 0; // Nothing has changed
					}
				}
			}
		}
		
		// Should not happen
		return 0.0;
	}
	
	public void resetRecognizer() {
		recognizer_.reset();
	}
	
	// Getters, etc.
	
	public String getLexicalForm() {
		return lexicalForm_;
	}

	public Signature getSignature() {
		return signature_;
	}

	public boolean hasSignature() {
		return signature_ != null;
	}
	
	public boolean hasFSM() {
		return fsm_ != null;
	}
	
	public boolean hasRecognizer() {
		return recognizer_ != null;
	}
	
	private void makeVerbFolder() {
		File verbFolder = new File("verbs/" + lexicalForm_);
		if (!verbFolder.exists()) {
			verbFolder.mkdirs();
		}
	}
	
	public String getVerbFolder() {
		return "verbs/" + lexicalForm_ + "/";
	}
}
