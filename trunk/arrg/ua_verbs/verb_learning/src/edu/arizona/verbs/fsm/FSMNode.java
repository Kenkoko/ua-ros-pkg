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
	private int verbSequenceIndex_ = 0;
	
	public FSMNode(StateType type) { 
		type_ = type; 
		id_ = counter++;
		if (type.equals(StateType.TERMINAL)) {
			minDist_ = 0;
		}
		makeString();
	}
	
	public FSMNode(StateType type, int sequenceIndex) { 
		type_ = type; 
		id_ = counter++;
		if (type.equals(StateType.TERMINAL)) {
			minDist_ = 0;
		}
		verbSequenceIndex_ = sequenceIndex;
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
	
	public void clearMinDist() {
		minDist_ = -1;
	}
	
	public void setMinDist(int minDist) {
		minDist_ = minDist;
		makeString();
	}
	
	public int getMinDist() {
		return minDist_;
	}
	
	public int getVerbSequenceIndex() {
		return verbSequenceIndex_;
	}
	
	public void setVerbSequenceIndex(int index) {
		verbSequenceIndex_ = index;
	}
	
	@Override
	public String toString() {
		return toString_; // Type is for debugging
	}
	
	public String toDot() {
		return "\t\"" + this.id_ + "\" [label=\"" + (this.toString() + minDist_ + "_" + verbSequenceIndex_) + "\""
				+ getDotString() + "];\n";
	}

	private String getDotString() {
		switch (type_) {
		case START:
			return ",shape=\"circle\",fontcolor=\"" + "black" + "\"";
		case INTERIOR:
			return ",shape=\"circle\",fontcolor=\"" + "black" + "\"";
		case TERMINAL:
			return ",shape=\"doublecircle\",fontcolor=\"" + "black" + "\"";
		default:
			return "magenta"; // Because magenta is never the right color
		}
		
		
	}
	
	@Override
	public int compareTo(FSMNode other) {
		return toString().compareTo(other.toString());
	}
	
	private void makeString() {
		toString_ = String.valueOf(id_) + type_.toString();
	}
}
