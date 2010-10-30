package edu.arizona.verbs.fsm;

import java.util.HashSet;
import java.util.Set;
import java.util.TreeSet;

import edu.arizona.verbs.fsm.FSMState.StateType;

public class StateSet {
	public static boolean containsGoodTerminal(Set<FSMState> state) {
		Set<StateType> types = new HashSet<FSMState.StateType>();
		for (FSMState s : state) {
			types.add(s.getType());
		}
		
		return types.contains(StateType.GOOD_TERMINAL);
	}
	
	public static StateType getStateType(TreeSet<FSMState> state) {
		Set<StateType> types = new HashSet<FSMState.StateType>();
		for (FSMState s : state) {
			types.add(s.getType());
		}
		
		if (types.contains(StateType.BAD_TERMINAL)) {
			return StateType.BAD_TERMINAL;
		} else if (types.contains(StateType.GOOD_TERMINAL)) {
			return StateType.GOOD_TERMINAL;
		} else if (types.contains(StateType.GOOD)) {
			return StateType.GOOD;
		} else {
			return StateType.START;
		}
 	}
}
