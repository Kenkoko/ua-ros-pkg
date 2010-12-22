package edu.arizona.verbs.fsm;

import edu.arizona.verbs.util.Predicate;

public class FSMNode implements Comparable<FSMNode> {
	private static int counter = 0; 
	
	// Predicates
	public static Predicate<FSMNode> isTerminal = new Predicate<FSMNode>() {
		@Override
		public boolean value(FSMNode state) {
			return state.isTerminal();
		};
	};
	
	public enum StateType { START, INTERIOR, TERMINAL }; 
	
	private StateType type_;
	private int id_;
	private int minDist_ = -1; // The minimum distance to a good terminal
	private String toString_;
	
	public FSMNode(StateType type) { 
		type_ = type; 
		id_ = counter++;
		if (type.equals(StateType.TERMINAL)) {
			minDist_ = 0;
		}
		makeString();
	}
	
	private String getColor() {
		switch (type_) {
		case START:
			return "blue";
		case INTERIOR:
			return "white";
		case TERMINAL:
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
		return (getType().equals(StateType.TERMINAL));
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
	public int compareTo(FSMNode other) {
		return toString().compareTo(other.toString());
	}
	
	private void makeString() {
		toString_ = String.valueOf(id_) + type_.toString();
	}
}
