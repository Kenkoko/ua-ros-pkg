package edu.arizona.verbs.verb.vfsm;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Set;

import edu.arizona.cs.learn.timeseries.model.Signature;
import edu.arizona.verbs.fsm.FSMNode.StateType;
import edu.arizona.verbs.fsm.VerbFSM;
import edu.arizona.verbs.fsm.VerbFSM.TransitionResult;

public class AtomicVerb extends AbstractVerb {
//	private static Logger logger = Logger.getLogger(AtomicVerb.class);
	
	// Used internally for making a remapped verb
	private AtomicVerb() {
	}
	
	public AtomicVerb(String word) {
		lexicalForm_ = word;
		makeVerbFolder();
	}
	
	public AtomicVerb(String word, ArrayList<String> arguments) {
		lexicalForm_ = word;
		arguments_ = arguments;
		makeVerbFolder();
	}
	
	public AtomicVerb(String word, List<String> arguments, Signature signature, Signature negSignature) {
		lexicalForm_ = word;
		arguments_ = new ArrayList<String>(arguments);
		makeVerbFolder();
		postInstance();
	}
	
	@Override
	protected void postInstance() {
		makeFSM();
	}
	
	private void makeFSM() {
		posFSM_ = new VerbFSM(posCorePaths_);
		negFSM_ = new VerbFSM(negCorePaths_);
		
		posFSM_.toDot(getVerbFolder() + "pos.dot", false);
		negFSM_.toDot(getVerbFolder() + "neg.dot", false);
	}
	
//	public void resetRecognizer() {
//		posFSM_.reset();
//		negFSM_.reset();
//	}

	@Override
	public FSMVerb remap(Map<String, String> nameMap) {
		AtomicVerb newVerb = new AtomicVerb();
		newVerb.lexicalForm_ = lexicalForm_;
		newVerb.arguments_ = new ArrayList<String>();
		for (String s : arguments_) {
			newVerb.arguments_.add(nameMap.containsKey(s) ? nameMap.get(s) : s);
		}
		newVerb.baseVerb_ = this;
		newVerb.posFSM_ = posFSM_.remap(nameMap);
		newVerb.negFSM_ = negFSM_.remap(nameMap);
		
		newVerb.posFSM_.toDot("pos-remap.dot", false);
		newVerb.negFSM_.toDot("neg-remap.dot", false);
		
		return newVerb;
	}

	@Override
	public int getHeuristic(VerbState verbState) {
		if (verbState.isBadTerminal()) {
			return Integer.MAX_VALUE;
		}
		return posFSM_.getMinDistToTerminal(verbState.posState);
	}

	@Override
	public VerbState fsmTransition(VerbState verbState, Set<String> activeRelations) {
		TransitionResult posResult = posFSM_.simulateDfaTransition(verbState.posState, activeRelations);
//		TransitionResult posResult = posFSM_.simulateNfaTransition(verbState.posState, activeRelations);
		
		TransitionResult negResult = negFSM_.simulateNfaTransition(verbState.negState, activeRelations);
		
		return new VerbState(posResult.newState, negResult.newState);
	}
}
