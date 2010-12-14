package edu.arizona.verbs.verb;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintStream;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.Map.Entry;
import java.util.TreeSet;
import java.util.Vector;

import org.yaml.snakeyaml.Yaml;

import com.google.common.base.Joiner;
import com.google.common.collect.Collections2;
import com.google.common.collect.Lists;
import com.google.common.collect.Maps;

import ros.pkg.oomdp_msgs.msg.MDPState;
import ros.pkg.verb_learning.srv.PerformVerb;
import ros.pkg.verb_learning.srv.PerformVerb.Response;
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
import edu.arizona.verbs.planning.shared.Action;
import edu.arizona.verbs.shared.OOMDPState;
import edu.uci.ics.jung.graph.DirectedGraph;

public class SequentialVerb extends AbstractVerb {
	private Vector<VerbBinding> verbBindings_ = new Vector<VerbBinding>();
	
	public SequentialVerb(String lexicalForm, List<String> arguments, List<VerbBinding> verbs) {
		lexicalForm_ = lexicalForm;
		arguments_ = new ArrayList<String>(arguments);
		verbBindings_.addAll(verbs);
		
		initializeSignatures();
		makeVerbFolder();
		
		File f = new File(getVerbFolder() + "sequential.yaml");
		if (!f.exists()) {
			try {
				f.createNewFile(); 
				PrintStream out = new PrintStream(f);
				List<Object> yamlList = new Vector<Object>();
				for (VerbBinding vb : verbBindings_) {
					Map<String,Map<String,String>> yamlMap = Maps.newLinkedHashMap();
					yamlMap.put(vb.verb.getIdentifierString(), vb.binding);
					yamlList.add(yamlMap);
				}
				
				Yaml yaml = new Yaml();
				String dump = yaml.dump(yamlList);
				System.out.println(dump);
				out.print(dump);
			} catch (IOException e) {
				e.printStackTrace();
			}
		}
		
		generateFSM(); 
	}
	
	private SequentialVerb() {
	}
	
	private void generateFSM() {
		Vector<Verb> verbs = new Vector<Verb>();
		for (VerbBinding vb : verbBindings_) {
			Verb subverb = vb.verb.remap(vb.binding);
			verbs.add(subverb);
		}
		
		fsm_ = null; // Delete the old fsm. One of the subverbs might have been updated
		for (Verb verb : verbs) {
			VerbFSM next = verb.getFSM();
			if (fsm_ == null) {
				fsm_ = next.duplicate();
			} else {
				fsm_ = fsm_.concatenate(next);
			}
		}
		
		// TODO: Should getVerbFolder always just find the base verb?
		fsm_.toDot(getBaseVerb().getVerbFolder() + "intermediate.dot", false);
		
		// Now need to add the signature-based FSM (based on instances specific for this verb)
		
		Signature pruned = signature_; // NO PRUNING FOR NOW
		Set<String> propSet = new TreeSet<String>();
		for (WeightedObject obj : pruned.signature()) {
			propSet.addAll(obj.key().getProps());
		}
		List<String> props = new ArrayList<String>(propSet);
		List<List<Interval>> all = BitPatternGeneration.getBPPs(lexicalForm_, pruned.table(), propSet);

		DirectedGraph<BPPNode, Edge> chains = FSMFactory.makeGraph(props, all, false); 
//		FSMFactory.toDot(chains, getVerbFolder() + "chain_nfa.dot");
	
		/* NEGATIVE */
		Set<String> negPropSet = new TreeSet<String>();
		for (WeightedObject obj : negativeSignature_.signature()) {
			negPropSet.addAll(obj.key().getProps());
		}
		List<String> negProps = new ArrayList<String>(negPropSet);
		List<List<Interval>> allNeg = BitPatternGeneration.getBPPs(lexicalForm_, negativeSignature_.table(), negPropSet);

		DirectedGraph<BPPNode, Edge> negChains = FSMFactory.makeGraph(negProps, allNeg, false); 
//		FSMFactory.toDot(negChains, getVerbFolder() + "neg_chain_nfa.dot");
		
		// TODO: This is not getting triggered, why?
		if (signature_.trainingSize() > 0 || negativeSignature_.trainingSize() > 0) {
			VerbFSM sigFSM = new VerbFSM(chains, negChains);
			sigFSM.toDot(getBaseVerb().getVerbFolder() + "sig_dfa.dot", false);
			
			fsm_ = fsm_.alternate(sigFSM);
		}
		
		fsm_.toDot(getVerbFolder() + "final.dot", false);
	}
	
	@Override
	public Response perform(MDPState startState, int executionLimit) {
		PerformVerb.Response response = new PerformVerb.Response();
		
		if (!hasFSM()) {
			return response; // Automatic failure
		}
		
		OOMDPState start = StateConverter.msgToState(startState);
		// We will plan with the general names since that's what our verb FSM contains
		LRTDP planner = new LRTDP(this, Interface.getCurrentEnvironment());
		
		TreeSet<FSMState> fsmState = fsm_.getStartState();
		TransitionResult tr = fsm_.simulateDfaTransition(fsmState, start.getActiveRelations());
		fsmState = tr.newState;

		List<OOMDPState> trace = new Vector<OOMDPState>();
		trace.add(start);
		
//		String action = planner.runAlgorithm(start, fsmState);
		String action = "TERMINATE"; // TODO: If atomic works, copy here
		int numSteps = 0;
//		while ( numSteps < executionLimit && action != null && 
//				!action.toString().equals(Action.TERMINATE)) {
//			// Perform the action, get the new world state
//			OOMDPState newState = Interface.getCurrentEnvironment().performAction(action);
//			trace.add(newState);
//			numSteps++;
//			// Get the new FSM state
//			fsmState = fsm_.simulateDfaTransition(fsmState, newState.getActiveRelations()).newState;
//			
//			// Get the new action
//			action = planner.runAlgorithm(newState, fsmState);
//		}
		
		// TODO: Don't like having to duplicate this code, need to fix that
		response.trace = StateConverter.stateToMsgArray(trace);
		response.execution_success = (action != null && action.toString().equals(Action.TERMINATE));
		response.execution_length = numSteps;
		response.planning_time = 0; // TODO: Compute the total planning time
		
		return response;
	}

	@Override
	public Verb remap(Map<String, String> nameMap) {
		SequentialVerb remapped = new SequentialVerb();
		
		remapped.lexicalForm_ = lexicalForm_;
		remapped.arguments_ = new ArrayList<String>();
		for (String s : arguments_) {
			remapped.arguments_.add((nameMap.containsKey(s) ? nameMap.get(s) : s));
		}
		remapped.baseVerb_ = this;
		
//		for (VerbBinding vb : verbBindings_) {
//			VerbBinding rebinding = new VerbBinding();
//			rebinding.verb = vb.verb;
//			for (Entry<String,String> entry : vb.binding.entrySet()) {
//				rebinding.binding.put(entry.getKey(), 
//						(nameMap.containsKey(entry.getValue()) ? nameMap.get(entry.getValue()) : entry.getValue()));
//			}
//			
//			remapped.verbBindings_.add(rebinding);
//		}
//		// Alternately, we could directly remap the FSM here, rather than regenerate it.
//		remapped.generateFSM();
		
		remapped.fsm_ = fsm_.remap(nameMap);
		
		return remapped;
	}

	@Override
	protected void postInstance() {
		// Saving signatures for later
		if (signature_.trainingSize() > 0) {
			String filename = getVerbFolder() + "signature.xml"; 
			signature_.toXML(filename);
		}

		if (negativeSignature_.trainingSize() > 0) {
			String negFilename = getVerbFolder() + "neg-signature.xml";
			negativeSignature_.toXML(negFilename);
		}
		
		generateFSM();
	}
}
