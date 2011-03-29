package edu.arizona.verbs.verb.bayes;

import java.util.ArrayList;

import edu.arizona.verbs.shared.OOMDPState;

public class SimplePolicy {
	private ArrayList<OOMDPState> mdpStates = new ArrayList<OOMDPState>();
	private ArrayList<String> actions = new ArrayList<String>();

	public SimplePolicy() {}
	
	public SimplePolicy(SimplePolicy sp) {
		this.mdpStates.addAll(sp.mdpStates);
		this.actions.addAll(sp.actions);
	}
	
	public void setStartState(OOMDPState state) {
		mdpStates.add(state);
	}
	
	// This means you took action "action" and it took you to state "state"
	public void addActionState(String action, OOMDPState state) {
		actions.add(action);
		mdpStates.add(state);
	}
	
	public void finish() {
		actions.add("TERMINATE");
	}
	
	public void replan() {
		actions.add("REPLAN");
	}
	
	public void stripLast() {
		if (!actions.isEmpty())
			actions.remove(actions.size()-1);
	}

	public ArrayList<OOMDPState> getMdpStates() {
		return mdpStates;
	}

	public ArrayList<String> getActions() {
		return actions;
	}
	
	public void print() {
		System.out.println("POLICY:");
		for (int i = 0; i < mdpStates.size(); i++) {
			System.out.println(mdpStates.get(i));
			System.out.println("--- > " + actions.get(i));
		}
	}
}
