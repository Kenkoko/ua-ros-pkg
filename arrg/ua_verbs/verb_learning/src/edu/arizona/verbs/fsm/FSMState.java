package edu.arizona.verbs.fsm;

import java.util.Collection;
import java.util.Collections;
import java.util.HashSet;
import java.util.Set;
import java.util.SortedSet;
import java.util.TreeSet;

import edu.arizona.verbs.fsm.FSMNode.StateType;

/**
 * @author Daniel Hewlett
 * 
 * This class provides an immutable representation of the current state of an FSM 
 * as a set of FSMNodes, thus allowing for NFA-style transitions.
 * 
 */
public class FSMState {
	private TreeSet<FSMNode> subStates_ = new TreeSet<FSMNode>();
	
	public FSMState() {
		// Does nothing
	}
	
	public FSMState(FSMNode state) {
		subStates_.add(state);
	}
	
	public FSMState(Collection<FSMNode> states) {
		subStates_.addAll(states);
	}
	
	public SortedSet<FSMNode> getStates() {
		return Collections.unmodifiableSortedSet(subStates_); 
	}
	
	public boolean containsTerminal() {
		Set<StateType> types = new HashSet<FSMNode.StateType>();
		for (FSMNode s : subStates_) {
			types.add(s.getType());
		}
		
		return types.contains(StateType.TERMINAL);
	}
	
	public StateType getStateType() {
		Set<StateType> types = new HashSet<FSMNode.StateType>();
		for (FSMNode s : subStates_) {
			types.add(s.getType());
		}
		
		if (types.contains(StateType.TERMINAL)) {
			return StateType.TERMINAL;
		} else if (types.contains(StateType.INTERIOR)) {
			return StateType.INTERIOR;
		} else {
			return StateType.START;
		}
 	}

	@Override
	public int hashCode() {
		return subStates_.hashCode();
	}

	@Override
	public boolean equals(Object obj) {
		return (obj instanceof FSMState &&
				subStates_.equals(((FSMState) obj).subStates_));
	}
	
	public String toString() {
		return subStates_.toString();
	}
}
