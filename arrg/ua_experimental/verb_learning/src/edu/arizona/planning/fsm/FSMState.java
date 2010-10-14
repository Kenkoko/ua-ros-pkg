package edu.arizona.planning.fsm;

import edu.arizona.util.Predicate;

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
	
	public FSMState(StateType type) { 
		type_ = type; 
		id_ = counter++;
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
	}
	
	public boolean isTerminal() {
		return (getType().equals(StateType.GOOD_TERMINAL) || getType().equals(StateType.BAD_TERMINAL));
	}
	
	public int getID() {
		return id_;
	}
	
	@Override
	public String toString() {
		return String.valueOf(id_) + type_.toString(); // Type is for debugging
	}
	
	public String toDot() {
		return "\t\"" + this.id_ + "\" [label=\"" + (this.toString())
				+ "\",style=\"filled\",color=\"" + getColor()
				+ "\",fontcolor=\"" + "black" + "\"];\n";
	}

	@Override
	public int compareTo(FSMState other) {
		return toString().compareTo(other.toString());
	}
}
