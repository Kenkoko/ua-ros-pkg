package edu.arizona.verbs.planning.shared;

import java.util.List;

import edu.arizona.verbs.planning.data.PlanningReport;
import edu.arizona.verbs.planning.data.SimulationResult;
import edu.arizona.verbs.planning.state.PlanningState;
import edu.arizona.verbs.shared.OOMDPState;
import edu.arizona.verbs.verb.Verb;
import edu.arizona.verbs.verb.VerbState;

public interface Planner {
	
	public List<Action> getActions();
	
	public Verb getVerb();
	
	public PlanningReport runAlgorithm(OOMDPState startState, VerbState verbState);

	public void setMaxDepth(int maxDepth);

	public SimulationResult sampleNextState(PlanningState state, Action a);
}
