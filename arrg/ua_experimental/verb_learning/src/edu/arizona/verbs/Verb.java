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
import ros.pkg.verb_learning.msg.VerbDescription;
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
import edu.arizona.verbs.fsm.VerbFSM;
import edu.arizona.verbs.mdp.OOMDPState;
import edu.arizona.verbs.mdp.StateConverter;
import edu.arizona.verbs.planning.LRTDP;
import edu.uci.ics.jung.graph.DirectedGraph;

public class Verb {
	private static Logger logger = Logger.getLogger(Verb.class);
	
	private String lexicalForm_;
	private List<String> arguments_;
	private Signature signature_ = null;
	private Signature negativeSignature_; // This is not really used for pruning, just fits better
	private VerbFSM fsm_ = null;
	
	private boolean frozen = false;
	
	public Verb(String word) {
		lexicalForm_ = word;
		makeVerbFolder();

		inializeSignatures();
	}
	
	public Verb(String word, String[] arguments) {
		lexicalForm_ = word;
		arguments_ = Arrays.asList(arguments);
		makeVerbFolder();
		
		inializeSignatures();
	}
	
	public Verb(String word, String[] arguments, Signature signature, Signature negSignature) {
		lexicalForm_ = word;
		arguments_ = Arrays.asList(arguments);
		makeVerbFolder();

		signature_ = signature;
		negativeSignature_ = negSignature;
		
		postInstance();
	}
	
	private void inializeSignatures() {
		signature_ = new Signature(lexicalForm_);
		negativeSignature_ = new Signature("non-" + lexicalForm_);
	}
	
	public void addPositiveInstances(List<Instance> instances) {
		if (frozen) { return; }
		for (Instance instance : instances) {
			signature_.update(instance.sequence());
		}

		postInstance();
	}
	
	public void addPositiveInstance(Instance instance) {
		if (frozen) { return; }
		signature_.update(instance.sequence());
		
		postInstance();
	}
	
	public void addNegativeInstance(Instance instance) {
		if (frozen) { return; }
		negativeSignature_.update(instance.sequence());
		
		postInstance();
	}
	
	public void addNegativeInstances(List<Instance> instances) {
		if (frozen) { return; }
		for (Instance instance : instances) {
			negativeSignature_.update(instance.sequence());
		}
		
		postInstance();
	}
	
	private void postInstance() {
		if (frozen) { return; }
		
		// Let's only print the relations that are in all the episodes
		Signature consensus = signature_.prune(signature_.trainingSize() - 1);
		
		for (WeightedObject obj : consensus.signature()) {
			logger.debug("\t" + obj.key().getKey() + " - " + obj.weight());
		}

		// Saving signatures for later
		if (signature_.trainingSize() > 0) {
			String filename = getVerbFolder() + "signature.xml"; 
			signature_.toXML(filename);
			logger.debug("Signature saved to file: " + filename);
		}

		if (negativeSignature_.trainingSize() > 0) {
			String negFilename = getVerbFolder() + "neg-signature.xml";
			negativeSignature_.toXML(negFilename);
			logger.debug("Negative signature saved to file: " + negFilename);
		}
		
		makeFSM();
	}
	
	public void forgetInstances() {
		if (frozen) { return; }
		signature_ = new Signature(lexicalForm_);
		negativeSignature_ = new Signature("non-" + lexicalForm_);
	}
	
	private void makeFSM() {
		if (!hasSignature()) {
			logger.error("WTF are you doing? There is no signature for " + lexicalForm_);
			return;
		}
		
		/* POSITIVE */
//		Signature pruned = signature_.prune(signature_.trainingSize() - 1);
//		Signature pruned = signature_.prune(signature_.trainingSize() / 2);
		Signature pruned = signature_; // NO PRUNING FOR NOW
		Set<String> propSet = new TreeSet<String>();
		for (WeightedObject obj : pruned.signature()) {
			propSet.addAll(obj.key().getProps());
		}
		List<String> props = new ArrayList<String>(propSet);
		List<List<Interval>> all = BitPatternGeneration.getBPPs(lexicalForm_, pruned.table(), propSet);

		DirectedGraph<BPPNode, Edge> chains = FSMFactory.makeGraph(props, all, false); 
		FSMFactory.toDot(chains, getVerbFolder() + "chain_nfa.dot");
	
		/* NEGATIVE */
		Set<String> negPropSet = new TreeSet<String>();
		for (WeightedObject obj : negativeSignature_.signature()) {
			negPropSet.addAll(obj.key().getProps());
		}
		List<String> negProps = new ArrayList<String>(negPropSet);
		List<List<Interval>> allNeg = BitPatternGeneration.getBPPs(lexicalForm_, negativeSignature_.table(), negPropSet);

		DirectedGraph<BPPNode, Edge> negChains = FSMFactory.makeGraph(negProps, allNeg, false); 
		FSMFactory.toDot(negChains, getVerbFolder() + "neg_chain_nfa.dot");
		
		fsm_ = new VerbFSM(chains, negChains);
		fsm_.toDot(getVerbFolder() + "dfa.dot", false);
	}
	
	public void addConstraint(Collection<String> bannedProps) {
		fsm_.addConstraintState(new HashSet<String>(bannedProps));
		fsm_.toDot(getVerbFolder() + "constrained.dot", false);
	}
	
	// TODO: Really should return a new verb, but then we need to make a new DFA
	public void concatenate(Verb other, String newName, String[] newArguments) {
		fsm_.concatenate(other.fsm_);
		lexicalForm_ = newName;
		arguments_ = Arrays.asList(newArguments);
		frozen = true;
		other.frozen = true; // Don't take any chances
	}
	
	@Deprecated
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
		return fsm_.transitionDFA(activeProps);
	}
	
	// argumentMap maps the concrete names to the specific ones
	// success in the response is only false if the runTrial call runs "forever"
	public Response planVerb(MDPState startState, Map<String, String> argumentMap) {
		Response result = new Response();
		if (!hasFSM()) {
			result.success = 0; // Automatic failure
			return result;
		}
		
		OOMDPState properStart = StateConverter.msgToState(startState);
		// We will plan with the general names since that's what our verb FSM contains
		OOMDPState remappedStart = properStart.remapState(argumentMap);
		
		LRTDP planner = new LRTDP(this, Interface.getCurrentEnvironment(), remappedStart);
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
		fsm_.reset();
	}
	
	// Getters, etc.
	
	public String getLexicalForm() {
		return lexicalForm_;
	}

	public Signature getSignature() {
		return signature_;
	}

	public VerbFSM getFSM() {
		return fsm_;
	}
	
	public boolean hasSignature() {
		return signature_ != null;
	}
	
	public boolean hasFSM() {
		return fsm_ != null;
	}
	
	public String[] getArgumentArray() {
		return arguments_.toArray(new String[0]);
	}
	
	private void makeVerbFolder() {
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
	}
	
	public VerbDescription makeVerbDescription() {
		VerbDescription desc = new VerbDescription();
		desc.verb = lexicalForm_;
		desc.arguments = arguments_.toArray(new String[0]);
		return desc;
	}
}
