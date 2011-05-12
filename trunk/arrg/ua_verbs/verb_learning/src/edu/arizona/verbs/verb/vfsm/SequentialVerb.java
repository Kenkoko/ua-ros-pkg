package edu.arizona.verbs.verb.vfsm;

import java.io.File;
import java.io.IOException;
import java.io.PrintStream;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.Set;
import java.util.Vector;

import org.yaml.snakeyaml.Yaml;

import com.google.common.collect.Maps;

import edu.arizona.verbs.fsm.VerbFSM;
import edu.arizona.verbs.fsm.VerbFSM.TransitionResult;
import edu.arizona.verbs.verb.VerbBinding;
import edu.arizona.verbs.verb.vfsm.learner.CorePathLearner;

// TODO: This isn't quite right anymore since the core paths change, fix it
public class SequentialVerb extends AbstractVerb {
//	private static Logger logger = Logger.getLogger(SequentialVerb.class);
	
	private Vector<VerbBinding> verbBindings_ = new Vector<VerbBinding>();
	
	public SequentialVerb(String lexicalForm, List<String> arguments, List<VerbBinding> verbs) {
		lexicalForm_ = lexicalForm;
		arguments_ = new ArrayList<String>(arguments);
		verbBindings_.addAll(verbs);
		
		// TODO: This should not be hardcoded
		posLearner_ = new CorePathLearner();
		negLearner_ = new CorePathLearner();
		
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
	
	public void makeFSM() {
		if (verbBindings_.isEmpty()) {
			throw new RuntimeException("Invalid Sequential Verb: No subverbs!");
		}
		
		Vector<FSMVerb> verbs = new Vector<FSMVerb>();
		for (VerbBinding vb : verbBindings_) {
			FSMVerb subverb = vb.verb.remap(vb.binding);
			verbs.add(subverb);
		}

		// Note: The positive FSM is the concatentation of all the subverbs
		// positive FSM. The negative FSM needs to be handled differently
		posFSM_ = null; // Delete the old fsm. One of the subverbs might have been updated
		for (FSMVerb verb : verbs) {
			if (verb.hasPositiveFSM()) {
				VerbFSM next = verb.getPositiveFSM();
				if (posFSM_ == null) {
					posFSM_ = next.duplicate();
				} else {
					posFSM_ = posFSM_.concatenate(next);
				}
			}
		}
		
		if (hasPositiveFSM()) {
			posFSM_.toDot(getBaseVerb().getVerbFolder() + "final.dot", false);
		}
		
//		posFSM_.toDot(getBaseVerb().getVerbFolder() + "intermediate.dot", false);
		
		// Now need to add in the FSM based on instances specific for this verb
		// TODO ADD IN, not replace!!
//		if (!posCorePaths_.isEmpty()) {
//			posFSM_ = new VerbFSM(posCorePaths_);
//		}
		
		
		// TODO Need to keep track of which subverb we are doing and use the appropriate neg fsm
		// For now we will just ignore the subverbs neg fsms and make our own
		negFSM_ = negLearner_.getFSM();
	}

	@Override
	public FSMVerb remap(Map<String, String> nameMap) {
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
		if (verbState.isBadTerminal()) {
			return Integer.MAX_VALUE;
		}
		return posFSM_.getMinDistToTerminal(verbState.posState);
	}

	// TODO: We can safely lift this now
	@Override
	public VerbState fsmTransition(VerbState verbState, Set<String> activeRelations) {
		// TODO: This method needs to use the correct negFSM dynamically based on the positive state
		// Probably need to make a map from pos state to neg fsms?
		TransitionResult posResult = posFSM_.simulateNfaTransition(verbState.posState, activeRelations);
		TransitionResult negResult = negFSM_.simulateNfaTransition(verbState.negState, activeRelations);
		
		return new VerbState(posResult.newState, negResult.newState);
	}
}
