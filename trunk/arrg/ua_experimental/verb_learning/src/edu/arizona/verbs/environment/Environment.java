package edu.arizona.verbs.environment;

import java.util.List;

import edu.arizona.verbs.mdp.OOMDPObjectState;
import edu.arizona.verbs.mdp.OOMDPState;

public interface Environment {
	
	// Return the list of possible actions
	public List<String> getActions();
	
	// Simulate taking an action from a particular state, return the new state
	public OOMDPState simulateAction(OOMDPState state, String action);

	/* The next three are not currently used in verb_learning,
	 *  but are part of the environment specification, so are included for completeness. */
	
	// Initialize the environment given a set of object states, return the full OOMDPState with relations
	public OOMDPState initializeEnvironment(List<OOMDPObjectState> objects);
	
	// Perform the action in the actual environment from the current state
	// Might be the same as simulate in some environments
	public OOMDPState performAction(String action);
	
	// Given a set of objects, compute the relations that are currently true
	public OOMDPState computeRelations(List<OOMDPObjectState> objects);
	
}
