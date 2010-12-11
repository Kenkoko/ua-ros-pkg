package edu.arizona.verbs.planning.shared;

import java.util.ArrayList;
import java.util.List;
import java.util.TreeSet;

import edu.arizona.verbs.fsm.FSMState;
import edu.arizona.verbs.planning.search.SearchNode;
import edu.arizona.verbs.planning.state.LRTDPState;
import edu.arizona.verbs.shared.OOMDPState;

public class Policy {
	private List<OOMDPState> mdpStates = new ArrayList<OOMDPState>();
	private List<TreeSet<FSMState>> fsmStates = new ArrayList<TreeSet<FSMState>>();
	private List<String> actions = new ArrayList<String>();
	private boolean fake = false;
	
	public Policy() { // This is the null policy
		fake = true;
	}
	
	public Policy(List<State> states, List<Action> actions) {
		for (int i = 0; i < states.size(); i++) {
			State state = states.get(i);
			
			mdpStates.add(state.getMdpState());
			fsmStates.add(state.getFsmState());
			this.actions.add(actions.get(i).toString());
		}
	}
	
	public Policy(List<SearchNode> path) {
		for (int i = 0; i < path.size(); i++) {
			SearchNode searchNode = path.get(i);
			
			mdpStates.add(searchNode.state.getMdpState());
			fsmStates.add(searchNode.state.getFsmState());
			
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
	public String getAction(OOMDPState mdpState, TreeSet<FSMState> fsmState) {
		if (fake) {
			return Action.TERMINATE;
		} else {
			for (int i = 0; i < mdpStates.size(); i++) {
				if (mdpState.toString().equals(mdpStates.get(i).toString()) 
					&& fsmState.toString().equals(fsmStates.get(i).toString())) {
					return actions.get(i);
				} 
			}
			return null;
		}
	}
}
