package edu.arizona.verbs.planning.shared;

/**
 * This is a stub in case future actions have arguments or are otherwise more complex than simple strings.
 * 
 * @author dhewlett
 */
public class Action implements Comparable<Action> {
	
	public static final String TERMINATE = "TERMINATE";
	public static final Action TERMINATE_ACTION = new Action("TERMINATE");
	
	public static final String REPLAN = "REPLAN";
	public static final Action REPLAN_ACTION = new Action("REPLAN");
	
	String name_;
	
	public Action(String name) {
		name_ = name;
	}
	
	@Override
	public boolean equals(Object arg0) {
		return (this.toString() == arg0.toString());
	}

	@Override
	public int hashCode() {
		return name_.hashCode();
	}

	@Override
	public String toString() {
		return name_;
	}

	@Override
	public int compareTo(Action arg0) {
		return name_.compareTo(arg0.name_);
	}
}