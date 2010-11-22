package edu.arizona.verbs.verb;

import java.util.List;

import ros.pkg.oomdp_msgs.msg.MDPState;
import ros.pkg.verb_learning.srv.PerformVerb.Response;
import edu.arizona.cs.learn.algorithm.alignment.model.Instance;
import edu.arizona.cs.learn.timeseries.model.Signature;
import edu.arizona.verbs.fsm.VerbFSM;
import edu.arizona.verbs.shared.Remappable;

public interface Verb extends Remappable<Verb> {

	// Updating the verb FSM/signature
	public void addPositiveInstances(List<Instance> instances);
	public void addPositiveInstance(Instance instance);
	public void addNegativeInstance(Instance instance);
	public void addNegativeInstances(List<Instance> instances);	
	
	// Clear the FSM
	public void forgetInstances();
	
	// Perform the verb
	public Response perform(MDPState startState, int executionLimit);
	
	// Getters/Setters
	public String getLexicalForm();
	public Signature getSignature();
	public VerbFSM getFSM();
	public String[] getArgumentArray();
	public String getIdentifierString();
}
