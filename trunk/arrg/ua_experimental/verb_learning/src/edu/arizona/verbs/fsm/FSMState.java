package edu.arizona.verbs.fsm;

import edu.arizona.verbs.util.Predicate;

public class FSMState implements Comparable<FSMState> {
	private static int counter = 0; 
	
	
	// Predicates
	public static Predicate<FSMState> isTerminal = new Predicate<FSMState>() {
		@Override
		public boolean value(FSMState state) {
			return state.isTerminal();
		};
	};
	
	public enum StateType { START, GOOD, GOOD_TERMINAL, BAD_TERMINAL }; 
	
	private StateType type_;
	private int id_;
	private int minDist_ = -1; // The minimum distance to a good terminal
	private String toString_;
	
	public FSMState(StateType type) { 
		type_ = type; 
		id_ = counter++;
		if (type.equals(StateType.BAD_TERMINAL)) {
			minDist_ = Integer.MAX_VALUE;
		} else if (type.equals(StateType.GOOD_TERMINAL)) {
			minDist_ = 0;
		}
		makeString();
	}
	
	private String getColor() {
		switch (type_) {
		case START:
			return "blue";
		case GOOD:
			return "white";
		case BAD_TERMINAL:
			return "red";
		case GOOD_TERMINAL:
			return "green";
		default:
			return "magenta"; // Because magenta is never the right color
		}
	}
	
	public StateType getType() {
		return type_;
	}

	public void setType(StateType type) {
		type_ = type;
		makeString();
	}
	
	public boolean isTerminal() {
		return (getType().equals(StateType.GOOD_TERMINAL) || getType().equals(StateType.BAD_TERMINAL));
	}
	
	public int getID() {
		return id_;
	}
	
	public boolean hasMinDist() {
		return (minDist_ != -1);
	}
	
	public void setMinDist(int minDist) {
		minDist_ = minDist;
		makeString();
	}
	
	public int getMinDist() {
		return minDist_;
	}
	
	@Override
	public String toString() {
		return toString_; // Type is for debugging
	}
	
	public String toDot() {
		return "\t\"" + this.id_ + "\" [label=\"" + (this.toString() + minDist_)
				+ "\",style=\"filled\",color=\"" + getColor()
				+ "\",fontcolor=\"" + "black" + "\"];\n";
	}

	@Override
	public int compareTo(FSMState other) {
		return toString().compareTo(other.toString());
	}
	
	private void makeString() {
		toString_ = String.valueOf(id_) + type_.toString();
	}
}
