package edu.arizona.verbs.experiments;

import java.util.List;

import edu.arizona.verbs.shared.OOMDPObjectState;

public class Scenario {
	public List<OOMDPObjectState> start;
	public List<String> actions;
	
	public List<OOMDPObjectState> getStart() {
		return start;
	}
	
	public void setStart(List<OOMDPObjectState> start) {
		this.start = start;
	}
	
	public List<String> getActions() {
		return actions;
	}
	
	public void setActions(List<String> actions) {
		this.actions = actions;
	}
}
