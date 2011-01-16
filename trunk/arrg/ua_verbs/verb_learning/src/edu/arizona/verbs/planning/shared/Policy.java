package edu.arizona.verbs.planning.shared;

import java.util.ArrayList;
import java.util.List;

import edu.arizona.verbs.planning.search.SearchNode;
import edu.arizona.verbs.planning.state.PlanningState;
import edu.arizona.verbs.shared.OOMDPState;
import edu.arizona.verbs.verb.vfsm.VerbState;

public class Policy {
	public enum PolicyType { Standard, Terminate, Replan };
	
	private List<OOMDPState> mdpStates = new ArrayList<OOMDPState>();
	private List<VerbState> verbStates = new ArrayList<VerbState>();
	private List<String> actions = new ArrayList<String>();
	private PolicyType type = PolicyType.Standard;
	
	public Policy(PolicyType pType) { // This is the null policy, signals immediate termination
		if (pType.equals(PolicyType.Standard)) {
			throw new RuntimeException("THIS CONSTRUCTOR IS NOT FOR STANDARD POLICIES");
		}
		type = pType;
	}
	
	public Policy(List<PlanningState> states, List<Action> actions) {
		for (int i = 0; i < states.size(); i++) {
			PlanningState state = states.get(i);
			
			mdpStates.add(state.getMdpState());
			verbStates.add(state.getVerbState());
			this.actions.add(actions.get(i).toString());
		}
	}
	
	public Policy(List<SearchNode> path) {
		for (int i = 0; i < path.size(); i++) {
			SearchNode searchNode = path.get(i);
			
			mdpStates.add(searchNode.state.getMdpState());
			verbStates.add(searchNode.state.getVerbState());
			
			if (i == path.size() - 1) {
				actions.add(Action.TERMINATE);
			} else {
				SearchNode succ = path.get(i + 1);
				for (Action a : searchNode.children.keySet()) {
					SearchNode child = searchNode.children.get(a);
					if (child.equals(succ)) {
						actions.add(a.toString());
						continue;
					}
				}
			}
		}
	}
	
	// Returns the appropriate action or null if the policy does not specify what to do from that state
	public String getAction(OOMDPState mdpState, VerbState verbState) {
		switch (type) {
		case Terminate:
			return Action.TERMINATE;
		case Replan:
			return Action.REPLAN;
		case Standard:
			for (int i = 0; i < mdpStates.size(); i++) {
				if (mdpState.toString().equals(mdpStates.get(i).toString()) 
					&& verbState.toString().equals(verbStates.get(i).toString())) {
					return actions.get(i);
				} 
			}
			return Action.REPLAN; // If agent fell off the path, he should replan
		default:
			throw new RuntimeException("IMPOSSIBLE POLICY TYPE");
		}
	}
}
