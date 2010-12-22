package edu.arizona.verbs.verb;

import java.util.ArrayList;
import java.util.List;
import java.util.Set;

import ros.pkg.oomdp_msgs.msg.MDPState;
import ros.pkg.verb_learning.srv.PerformVerb.Response;
import edu.arizona.verbs.fsm.VerbFSM;
import edu.arizona.verbs.shared.OOMDPState;
import edu.arizona.verbs.shared.Remappable;

public interface Verb extends Remappable<Verb> {

	// Updating the verb FSM/signature
	public void addPositiveInstance(List<OOMDPState> trace);
	public void addNegativeInstance(List<OOMDPState> trace);
	public void addPositiveInstances(List<List<OOMDPState>> traces);
	public void addNegativeInstances(List<List<OOMDPState>> traces);	
	
	// Clear the FSM
	public void forgetInstances();
	
	// Perform the verb
	public Response perform(MDPState startState, int executionLimit);
	
	// Getters/Setters
	public String getLexicalForm();
	public VerbFSM getPositiveFSM();
	public VerbFSM getNegativeFSM();
 	public String[] getArgumentArray();
	public ArrayList<String> getArguments();
	public String getIdentifierString();
	
	// Heuristic Function
	public int getHeuristic(VerbState verbState);
	public VerbState fsmTransition(VerbState verbState, Set<String> activeRelations);
	public VerbState getStartState();
	
	public boolean hasPositiveFSM();
	public boolean hasNegativeFSM();
}
