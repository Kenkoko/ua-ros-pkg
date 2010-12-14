package edu.arizona.verbs.verb;

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
import ros.pkg.verb_learning.srv.PerformVerb;
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
import edu.arizona.verbs.fsm.FSMState;
import edu.arizona.verbs.fsm.VerbFSM;
import edu.arizona.verbs.fsm.VerbFSM.TransitionResult;
import edu.arizona.verbs.main.Interface;
import edu.arizona.verbs.mdp.StateConverter;
import edu.arizona.verbs.planning.LRTDP;
import edu.arizona.verbs.planning.SearchPlanner;
import edu.arizona.verbs.planning.UCT;
import edu.arizona.verbs.planning.shared.Action;
import edu.arizona.verbs.planning.shared.Policy;
import edu.arizona.verbs.shared.OOMDPState;
import edu.uci.ics.jung.graph.DirectedGraph;

public class AtomicVerb extends AbstractVerb {
	private static Logger logger = Logger.getLogger(AtomicVerb.class);
	
	// Used internally for making a remapped verb
	private AtomicVerb() {
	}
	
	public AtomicVerb(String word) {
		lexicalForm_ = word;
		makeVerbFolder();

		initializeSignatures();
	}
	
	public AtomicVerb(String word, ArrayList<String> arguments) {
		lexicalForm_ = word;
		arguments_ = arguments;
		makeVerbFolder();
		
		initializeSignatures();
	}
	
	public AtomicVerb(String word, List<String> arguments, Signature signature, Signature negSignature) {
		lexicalForm_ = word;
		arguments_ = new ArrayList<String>(arguments);
		makeVerbFolder();

		signature_ = signature;
		negativeSignature_ = negSignature;
		
		postInstance();
	}
	
	@Override
	protected void postInstance() {
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
	
	// TODO: Move to higher level
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
	
	public PerformVerb.Response perform(MDPState startState, int executionLimit) {
		PerformVerb.Response response = new PerformVerb.Response();
		
		if (!hasFSM()) {
			return response; // Automatic failure
		}
		
		OOMDPState properStart = StateConverter.msgToState(startState);
		
//		long startTime = System.currentTimeMillis();
//		long elapsedTime = System.currentTimeMillis() - startTime;
		
		TreeSet<FSMState> fsmState = fsm_.getStartState();
		TransitionResult tr = fsm_.simulateDfaTransition(fsmState, properStart.getActiveRelations());
		fsmState = tr.newState;

		List<OOMDPState> trace = new Vector<OOMDPState>();
		System.out.println("START: " + properStart);
		trace.add(properStart);
		
		// TODO: Need to restore planning time information
		LRTDP planner = new LRTDP(this, Interface.getCurrentEnvironment());
//		SearchPlanner planner = new SearchPlanner(this, Interface.getCurrentEnvironment());
//		UCT planner = new UCT(this, Interface.getCurrentEnvironment(), executionLimit);
		Policy policy = planner.runAlgorithm(properStart, fsmState);
		String action = policy.getAction(properStart, fsmState); 

		int numSteps = 0;
		while ( numSteps < executionLimit && !action.toString().equals(Action.TERMINATE)) {
			// Perform the action, get the new world state
			OOMDPState newState = Interface.getCurrentEnvironment().performAction(action); 
			System.out.println("THIS: " + newState.getActiveRelations());
			trace.add(newState);
			numSteps++;
			// Get the new FSM state
			fsmState = fsm_.simulateDfaTransition(fsmState, newState.getActiveRelations()).newState;
			
			// Get the new action
			action = policy.getAction(newState, fsmState);
			if (action == null) { // This means we "fell off" the planned path
				// For UCT, we need a new instance for now
//				planner.setMaxDepth(executionLimit - numSteps);
				policy = planner.runAlgorithm(newState, fsmState); // re-plan
				action = policy.getAction(newState, fsmState);
			}
		}
		
		response.trace = StateConverter.stateToMsgArray(trace);
		response.execution_success = (action != null && action.toString().equals(Action.TERMINATE));
		response.execution_length = numSteps;
		response.planning_time = 0; // TODO: Compute the total planning time
		
		return response;
	}
	
	public void resetRecognizer() {
		fsm_.reset();
	}

	@Override
	public Verb remap(Map<String, String> nameMap) {
		AtomicVerb newVerb = new AtomicVerb();
		newVerb.lexicalForm_ = lexicalForm_;
		newVerb.arguments_ = new ArrayList<String>();
		for (String s : arguments_) {
			newVerb.arguments_.add(nameMap.containsKey(s) ? nameMap.get(s) : s);
		}
		newVerb.baseVerb_ = this;
		newVerb.fsm_ = fsm_.remap(nameMap);
		
		newVerb.fsm_.toDot("remap.dot", false);
		
		return newVerb;
	}
}
