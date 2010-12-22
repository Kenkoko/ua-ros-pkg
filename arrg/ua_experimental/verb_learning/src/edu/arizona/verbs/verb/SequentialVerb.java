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

import org.apache.log4j.Logger;
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
import edu.arizona.simulator.ww2d.external.WW2DEnvironment;
import edu.arizona.verbs.environments.GazeboEnvironment;
import edu.arizona.verbs.fsm.FSMState;
import edu.arizona.verbs.fsm.VerbFSM;
import edu.arizona.verbs.fsm.VerbFSM.TransitionResult;
import edu.arizona.verbs.main.Interface;
import edu.arizona.verbs.mdp.StateConverter;
import edu.arizona.verbs.planning.LRTDP;
import edu.arizona.verbs.planning.Planners;
import edu.arizona.verbs.planning.SearchPlanner;
import edu.arizona.verbs.planning.UCT;
import edu.arizona.verbs.planning.data.PlanningReport;
import edu.arizona.verbs.planning.shared.Action;
import edu.arizona.verbs.planning.shared.Planner;
import edu.arizona.verbs.planning.shared.Policy;
import edu.arizona.verbs.planning.shared.Policy.PolicyType;
import edu.arizona.verbs.shared.OOMDPState;
import edu.uci.ics.jung.graph.DirectedGraph;

public class SequentialVerb extends AbstractVerb {
	private static Logger logger = Logger.getLogger(SequentialVerb.class);
	
	private Vector<VerbBinding> verbBindings_ = new Vector<VerbBinding>();
	
	public SequentialVerb(String lexicalForm, List<String> arguments, List<VerbBinding> verbs) {
		lexicalForm_ = lexicalForm;
		arguments_ = new ArrayList<String>(arguments);
		verbBindings_.addAll(verbs);
		
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
		
		makeFSM(); 
	}
	
	private SequentialVerb() {
		// Empty constructor used by remap
	}
	
	private void makeFSM() {
		if (verbBindings_.isEmpty()) {
			throw new RuntimeException("Invalid Sequential Verb: No subverbs!");
		}
		
		Vector<Verb> verbs = new Vector<Verb>();
		for (VerbBinding vb : verbBindings_) {
			Verb subverb = vb.verb.remap(vb.binding);
			verbs.add(subverb);
		}

		// Note: The positive FSM is the concatentation of all the subverbs
		// positive FSM. The negative FSM needs to be handled differently
		posFSM_ = null; // Delete the old fsm. One of the subverbs might have been updated
		for (Verb verb : verbs) {
			VerbFSM next = verb.getPositiveFSM();
			if (posFSM_ == null) {
				posFSM_ = next.duplicate();
			} else {
				posFSM_ = posFSM_.concatenate(next);
			}
		}
		
		posFSM_.toDot(getBaseVerb().getVerbFolder() + "intermediate.dot", false);
		
		// Now need to add in the FSM based on instances specific for this verb
		if (!posCorePaths_.isEmpty()) {
			posFSM_ = new VerbFSM(posCorePaths_);
		}
		
		posFSM_.toDot(getBaseVerb().getVerbFolder() + "final.dot", false);
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
		
		for (VerbBinding vb : verbBindings_) {
			VerbBinding rebinding = new VerbBinding();
			rebinding.verb = vb.verb;
			for (Entry<String,String> entry : vb.binding.entrySet()) {
				rebinding.binding.put(entry.getKey(), 
						(nameMap.containsKey(entry.getValue()) ? nameMap.get(entry.getValue()) : entry.getValue()));
			}
			
			remapped.verbBindings_.add(rebinding);
		}
		remapped.makeFSM();
		
//		// Alternately, we could directly remap the FSM here, rather than regenerate it.
//		remapped.posFSM_ = posFSM_.remap(nameMap);
//		remapped.negFSM_ = negFSM_.remap(nameMap);
//		
//		remapped.posFSM_.toDot("pos-remap.dot", false);
//		remapped.negFSM_.toDot("neg-remap.dot", false);
		
		return remapped;
	}

	@Override
	protected void postInstance() {
		makeFSM();
	}

	@Override
	public int getHeuristic(VerbState verbState) {
		// TODO: Use the neg state also
		return posFSM_.getMinDistToTerminal(verbState.posState);
	}

	@Override
	public VerbState fsmTransition(VerbState verbState, Set<String> activeRelations) {
		// TODO: This method needs to use the correct negFSM dynamically based on the positive state
		// Probably need to make a map from pos state to neg fsms?
		
		TransitionResult posResult = posFSM_.simulateDfaTransition(verbState.posState, activeRelations);
		TransitionResult negResult = negFSM_.simulateNfaTransition(verbState.negState, activeRelations);
		
		return new VerbState(posResult.newState, negResult.newState);
	}
}
