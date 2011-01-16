package edu.arizona.verbs.verb.vfsm;

import java.util.Set;

import edu.arizona.verbs.fsm.VerbFSM;
import edu.arizona.verbs.shared.Remappable;
import edu.arizona.verbs.verb.Verb;

public interface FSMVerb extends Verb, Remappable<FSMVerb> {
	// Getters/Setters
	public VerbFSM getPositiveFSM();
	public VerbFSM getNegativeFSM();
	
	// Heuristic Function
	public int getHeuristic(VerbState verbState);
	public VerbState fsmTransition(VerbState verbState, Set<String> activeRelations);
	public VerbState getStartState();
	
	public boolean hasPositiveFSM();
	public boolean hasNegativeFSM();
}
