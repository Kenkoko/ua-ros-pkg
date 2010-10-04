package edu.arizona.planning.mdp;

import java.util.Arrays;
import java.util.Collections;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.Vector;

import ros.pkg.simulator_state.msg.SimpleRelation;
import edu.arizona.util.StateUtils;

public class MDPState {
	private List<MDPObjectState> objectStates_;
	private List<SimpleRelation> relations_;
	
	public MDPState(ros.pkg.wubble_mdp.msg.MDPState stateMessage) {
		objectStates_ = new Vector<MDPObjectState>();
		
		for (ros.pkg.wubble_mdp.msg.MDPObjectState objState : stateMessage.object_states) {
			objectStates_.add(MDPObjectState.parseObjectMessage(objState));
		}
		
		Collections.sort(objectStates_); // Important!
		
		relations_ = new Vector<SimpleRelation>();
		relations_.addAll(Arrays.asList(stateMessage.relations));
	}
	
	public MDPState(List<MDPObjectState> objectStates, List<SimpleRelation> relations) {
		objectStates_ = objectStates;
		
		Collections.sort(objectStates_); // Important!
		
		relations_ = relations;
	}
	
	public List<MDPObjectState> getObjectStates() {
		return objectStates_;
	}
	
	public Set<String> getActiveRelations() {
		Set<String> result = new HashSet<String>();
		
		for (SimpleRelation rel : relations_) {
			if (rel.value > 0) { // It's actually boolean
				result.add(StateUtils.formatRelation(rel));
			}
		}
		
		return result;
	}
	
	@Override
	public String toString() {
		String result = new String();
		
		for (MDPObjectState os : objectStates_) {
			result += os.toString();
		}
		
		for (SimpleRelation rel : relations_) {
			if (rel.value > 0) {
				result += StateUtils.formatRelation(rel);
			}
		}
		
		return result;
	}
	
	public boolean isOutOfBounds() {
		for (MDPObjectState obj : objectStates_) {
			if (Math.abs(obj.x) > 10 || Math.abs(obj.y) > 10) {
				return true;
			}
		}
		
		return false;
	}
}
