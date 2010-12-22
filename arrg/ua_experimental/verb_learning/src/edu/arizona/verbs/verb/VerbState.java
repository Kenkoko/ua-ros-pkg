package edu.arizona.verbs.verb;

import edu.arizona.verbs.fsm.FSMState;

public class VerbState {
	public FSMState posState;
	public FSMState negState;
	
	public VerbState(FSMState pos, FSMState neg) {
		posState = pos;
		negState = neg;
	}
	
	public boolean isTerminal() {
		return (posState.containsTerminal() || negState.containsTerminal());
	}
	
	// This is for a cost function defined as c(s,a,s') = c(s')
	public int getCost() {
		if (negState.containsTerminal()) {
			return 1000; // TODO: Provide a way to set c_max
		} else if (posState.containsTerminal()){
			return 0;
		} else {
			return 1;
		}
	}
	
	public boolean isGoodTerminal() {
		return !isBadTerminal() && posState.containsTerminal();
	}
	
	public boolean isBadTerminal() {
		return negState.containsTerminal();
	}
	
	@Override
	public String toString() {
		return posState.toString() + negState.toString();
	}
}
