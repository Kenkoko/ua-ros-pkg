package edu.arizona.verbs.verb;

import java.util.ArrayList;
import java.util.List;

import ros.pkg.verb_learning.srv.PerformVerb.Response;
import edu.arizona.verbs.shared.Environment;
import edu.arizona.verbs.shared.OOMDPState;

// TODO: What to do about remapping?
public interface Verb {
	// Updating the verb FSM/signature
	public void addPositiveInstance(List<OOMDPState> trace);
	public void addNegativeInstance(List<OOMDPState> trace);
	public void addPositiveInstances(List<List<OOMDPState>> traces);
	public void addNegativeInstances(List<List<OOMDPState>> traces);	
	
	// Clear the FSM
	public void forgetInstances();
	
	// Perform the verb
	public Response perform(Environment env, OOMDPState startState, int executionLimit);
	public boolean isReady();
	
	// Getters/Setters
	public String getLexicalForm();
 	public String[] getArgumentArray();
	public ArrayList<String> getArguments();
	public String getIdentifierString();
}
