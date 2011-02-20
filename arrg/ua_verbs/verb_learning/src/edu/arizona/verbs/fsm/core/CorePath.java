package edu.arizona.verbs.fsm.core;

import java.util.Arrays;
import java.util.List;
import java.util.Vector;

import edu.arizona.verbs.shared.OOMDPState;
import edu.arizona.verbs.shared.Relation;

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
		System.out.println("THIS:");
		print();
		System.out.println("OTHER:");
		other.print();
		
		if (other.states_.size() >= states_.size()) {
			System.out.println("DOES NOT CONTAIN");
			return false;
		}

		for (int i = 0; i < states_.size() - other.states_.size() + 1; i++) {
			List<RelationalState> subseq = states_.subList(i, i + other.states_.size());
			
			//System.out.println("IS A EQUAL TO B?");
			//System.out.println("A: " + subseq);
			//System.out.println("B: " + other.states_);
			//System.out.println(subseq.equals(other.states_));
			
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
				System.out.println("CONTAINS");
				return true;
			}
		}
		
		System.out.println("DOES NOT CONTAIN");
		return false;
	}
	
	public void print() {
		int[] widths = new int[states_.size()]; 
		int height = 0;
		
		Arrays.fill(widths, 1); // No zero-length array cells
		
		for (int i = 0; i < states_.size(); i++) {
			height = Math.max(height, states_.get(i).getRelations().size());
			
			for (Relation rel : states_.get(i).getRelations()) {
				widths[i] = Math.max(widths[i], rel.toString().length());
			}
		}
		
		for (int w : widths) { 
			for (int i = 0; i < w + 3; i++) {
				System.out.print("=");
			}
		}
		System.out.println();
		
		for (int j = 0; j < height; j++) {
			for (int i = 0; i < states_.size(); i++) {
				Relation[] relations = states_.get(i).getRelations().toArray(new Relation[0]);
				if (j < relations.length) {
					System.out.print(String.format("%1$-" + widths[i] + "s", relations[j].toString()));
				} else {
					System.out.print(String.format("%1$-" + widths[i] + "s", ""));
				}
				System.out.print(" | ");
			}
			System.out.println();
		}
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
