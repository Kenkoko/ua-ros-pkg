package edu.arizona.verbs;

import java.io.File;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.TreeSet;
import java.util.Vector;

import org.apache.log4j.Logger;

import ros.pkg.oomdp_msgs.msg.MDPState;
import ros.pkg.verb_learning.msg.Policy;
import ros.pkg.verb_learning.msg.VerbDescription;
import ros.pkg.verb_learning.srv.PlanVerb;
import ros.pkg.verb_learning.srv.PlanVerb.Response;

import com.google.common.base.Joiner;
import com.google.common.collect.HashBiMap;

import edu.arizona.cs.learn.algorithm.alignment.Params;
import edu.arizona.cs.learn.algorithm.alignment.SequenceAlignment;
import edu.arizona.cs.learn.algorithm.alignment.model.Instance;
import edu.arizona.cs.learn.algorithm.alignment.model.WeightedObject;
import edu.arizona.cs.learn.algorithm.markov.BPPNode;
import edu.arizona.cs.learn.algorithm.markov.FSMFactory;
import edu.arizona.cs.learn.timeseries.experiment.BitPatternGeneration;
import edu.arizona.cs.learn.timeseries.model.Interval;
import edu.arizona.cs.learn.timeseries.model.Signature;
import edu.arizona.cs.learn.util.graph.Edge;
import edu.arizona.environment.Environment;
import edu.arizona.planning.RTDP;
import edu.arizona.planning.fsm.VerbDFA;
import edu.arizona.planning.mdp.OOMDPState;
import edu.uci.ics.jung.graph.DirectedGraph;

public class Verb {
	private static Logger logger = Logger.getLogger(Verb.class);
	
	private String lexicalForm_;
	private List<String> arguments_;
	private Signature signature_ = null;
	private VerbDFA dfa_ = null;
	
	public Verb(String word) {
		lexicalForm_ = word;
		makeVerbFolder();
	}
	
	public Verb(String word, String[] arguments) {
		lexicalForm_ = word;
		arguments_ = Arrays.asList(arguments);
		makeVerbFolder();
	}
	
	public Verb(String word, String[] arguments, Signature signature) {
		lexicalForm_ = word;
		arguments_ = Arrays.asList(arguments);
		makeVerbFolder();

		signature_ = signature;
		makeDFA();
	}
	
	public VerbDescription makeVerbDescription() {
		VerbDescription desc = new VerbDescription();
		desc.verb = lexicalForm_;
		desc.arguments = arguments_.toArray(new String[0]);
		return desc;
	}
	
	public void generalizeInstances(List<Instance> instances) {
		signature_ = new Signature(lexicalForm_);
		for (Instance instance : instances) {
			signature_.update(instance.sequence());
		}

		postInstance();
	}
	
	public void addNewInstance(Instance instance) {
		if (signature_ == null) { // Safety
			signature_ = new Signature(lexicalForm_);
		}
		signature_.update(instance.sequence());
		
		postInstance();
	}
	
	public void addNegativeInstance() {
		
	}
	
	private void postInstance() {
		// Let's only print the relations that are in all the episodes
		Signature consensus = signature_.prune(signature_.trainingSize() - 1);
		
		for (WeightedObject obj : consensus.signature()) {
			logger.debug("\t" + obj.key().getKey() + " - " + obj.weight());
		}

		// This is not strictly necessary, but useful in case of crashes, etc.
		String filename = getVerbFolder() + "signature.xml"; 
		signature_.toXML(filename);
		logger.debug("Signature saved to file: " + filename);
		
		makeDFA();
	}
	
	public void forgetInstances() {
		
	}
	
	private void makeDFA() {
		if (!hasSignature()) {
			logger.error("WTF are you doing? There is no signature for " + lexicalForm_);
			return;
		}
		
//		Signature pruned = signature_.prune(signature_.trainingSize() - 1);
		Signature pruned = signature_.prune(signature_.trainingSize() / 2);
//		Signature pruned = signature_;
		Set<String> propSet = new TreeSet<String>();
		for (WeightedObject obj : pruned.signature()) {
			propSet.addAll(obj.key().getProps());
		}
		List<String> props = new ArrayList<String>(propSet);
		List<List<Interval>> all = BitPatternGeneration.getBPPs(lexicalForm_, pruned.table(), propSet);

		DirectedGraph<BPPNode, Edge> chains = FSMFactory.makeGraph(props, all, false); 
		FSMFactory.toDot(chains, getVerbFolder() + "chain_nfa.dot");
	
		dfa_ = new VerbDFA(chains);
		dfa_.toDot(getVerbFolder() + "dfa.dot", false);
	}
	
	public void addConstraint(Collection<String> bannedProps) {
		dfa_.addConstraintState(new HashSet<String>(bannedProps));
		dfa_.toDot(getVerbFolder() + "constrained.dot", false);
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
		return dfa_.update(activeProps);
	}
	
	// argumentMap maps the concrete names to the specific ones
	// success in the response is only false if the runTrial call runs "forever"
	public Response planVerb(MDPState startState, Map<String, String> argumentMap) {
		Response result = new Response();
		if (!hasDFA()) {
			result.success = 0; // Automatic failure
			return result;
		}
		
		OOMDPState properStart = new OOMDPState(startState);
		// We will plan with the general names since that's what our verb FSM contains
		OOMDPState remappedStart = OOMDPState.remapState(properStart, argumentMap);
		
		RTDP planner = new RTDP(this, Environment.getInstance(), remappedStart);
		long startTime = System.currentTimeMillis();
		boolean success = planner.runAlgorithm();
		long elapsedTime = System.currentTimeMillis() - startTime;
		
		if (success) {
			HashBiMap<String,String> biMap = HashBiMap.create(argumentMap); 
			result.plan = planner.recoverPolicy(biMap.inverse());
			result.success = 1;	
			result.time_millis = elapsedTime;
		} else {
			result.success = 0;
			result.time_millis = elapsedTime;
			// No plan
		}
		
		return result;
	}
	
	public void resetRecognizer() {
		dfa_.reset();
	}
	
	// Getters, etc.
	
	public String getLexicalForm() {
		return lexicalForm_;
	}

	public Signature getSignature() {
		return signature_;
	}

	public VerbDFA getDFA() {
		return dfa_;
	}
	
	public boolean hasSignature() {
		return signature_ != null;
	}
	
	public boolean hasDFA() {
		return dfa_ != null;
	}
	
	public String[] getArgumentArray() {
		return arguments_.toArray(new String[0]);
	}
	
	private void makeVerbFolder() {
//		File verbFolder = new File("verbs/" + lexicalForm_);
		File verbFolder = new File(getVerbFolder());
		if (!verbFolder.exists()) {
			verbFolder.mkdirs();
		}
	}
	
	public String getVerbFolder() {
		Vector<String> parts = new Vector<String>();
		parts.add(lexicalForm_);
		parts.addAll(arguments_);
		return "verbs/" + Joiner.on(",").join(parts) + "/";
//		return "verbs/" + lexicalForm_ + "/";
	}
}
