package edu.arizona.verbs.fsm.core;

import java.util.List;
import java.util.Vector;

import edu.arizona.verbs.shared.OOMDPState;

public class CorePath {

	private Vector<RelationalState> states_ = new Vector<RelationalState>();
	
	public CorePath(List<OOMDPState> trace) {
		// Compress the time series
		for (OOMDPState mdpState : trace) {
			RelationalState state = new RelationalState(mdpState.getRelations());
			
			if (states_.isEmpty()) {
				states_.add(state);
			} else if (!states_.lastElement().equals(state)) {
				states_.add(state);
			}
		}
	}
	
	public boolean contains(CorePath other) {
		if (other.states_.size() >= states_.size()) {
			return false;
		}

		for (int i = 0; i < states_.size() - other.states_.size() + 1; i++) {
			List<RelationalState> subseq = states_.subList(i, i + other.states_.size());
			
			System.out.println("IS A EQUAL TO B?");
			System.out.println("A: " + subseq);
			System.out.println("B: " + other.states_);
			System.out.println(subseq.equals(other.states_));
			
//			if (subseq.equals(other.states_)) {
//				return true;
//			}
			boolean result = true;
			for (int j = 0; j < subseq.size(); j++) {
				if (!subseq.get(j).isSuperset(other.states_.get(j))) {
					result = false;
				}
			}
			
			if (result) {
				return true;
			}
		}
		
		return false;
	}
	
	public Vector<RelationalState> getPath() {
		return states_;
	}

	@Override
	public int hashCode() {
		return states_.hashCode();
	}

	@Override
	public boolean equals(Object obj) {
		return (obj instanceof CorePath &&
				states_.equals(((CorePath) obj).states_));
	}
}
