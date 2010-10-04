package edu.arizona.planning.fsm;

import java.util.Collection;

import edu.arizona.cs.learn.algorithm.markov.BPPNode;

public class FSMState {
	private static int counter = 0; 
	
	public enum StateType { START, GOOD, GOAL, BAD, BAD_TERMINAL }; // Is this how these should be named in Java
	
	private BPPNode node_;
	private StateType type_;
	private int id_;
	
	public FSMState(StateType type) { // This one has no BPPNode, let's see how this works
		node_ = null;
		type_ = type; // Why do we even need the props?
//		deadState.color("red");
//		deadState.fontColor("white"); // TODO: Need to bring this back somehow.
		
		id_ = counter++;
	}
	
	public FSMState(BPPNode node, StateType type) {
		node_ = node;
		type_ = type;
		
		id_ = counter++;
	}
	
	// TODO: Let's hide this since we want to phase it out anyway
//	public BPPNode getBPPNode() {
//		return node_;
//	}
	
	private String getColor() {
		switch (type_) {
		case START:
			return "blue";
		case GOOD:
			return "white";
		case BAD:
			return "yellow";
		case BAD_TERMINAL:
			return "red";
		case GOAL:
			return "green";
		default:
			return "magenta"; // Because magenta is never the right color
		}
	}
	
	public StateType getType() {
		return type_;
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
	public boolean equals(Object obj) {
		if (obj instanceof FSMState) {
			return this.toString().equals(obj.toString());
		} else {
			return false;
		}
	}

	@Override
	public int hashCode() {
		return this.toString().hashCode();
	}
}
