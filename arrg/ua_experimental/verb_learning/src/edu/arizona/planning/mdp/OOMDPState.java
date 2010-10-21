package edu.arizona.planning.mdp;

import java.util.Arrays;
import java.util.Collections;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.Vector;

import com.google.common.base.Function;
import com.google.common.collect.Collections2;

import ros.pkg.oomdp_msgs.msg.MDPObjectState;
import ros.pkg.oomdp_msgs.msg.MDPState;
import ros.pkg.oomdp_msgs.msg.Relation;
import edu.arizona.util.StateUtils;

public class OOMDPState {
	
	public static OOMDPState remapState(OOMDPState state, Map<String,String> nameMap) {
		Vector<OOMDPObjectState> newObjects = new Vector<OOMDPObjectState>();
		for (OOMDPObjectState obj : state.objectStates_) {
			newObjects.add(OOMDPObjectState.remapState(obj, nameMap));
		}
		Vector<Relation> newRelations = new Vector<Relation>();
		for (Relation rel : state.relations_) {
			Relation newRel = new Relation();
			newRel.relation = rel.relation;
			newRel.value = rel.value;
			newRel.obj_names = new String[rel.obj_names.length];
			for (int i = 0; i < rel.obj_names.length; i++) {
				newRel.obj_names[i] = (nameMap.containsKey(rel.obj_names[i]) ? nameMap.get(rel.obj_names[i]) : rel.obj_names[i]); 
			}
			newRelations.add(newRel);
		}
		
		OOMDPState newState = new OOMDPState(newObjects, newRelations);
		return newState;
	}
	
	private List<OOMDPObjectState> objectStates_;
	private List<Relation> relations_;
	private String hashString_ = null;
	
	public OOMDPState(MDPState stateMessage) {
		objectStates_ = new Vector<OOMDPObjectState>();
		
		for (MDPObjectState objState : stateMessage.object_states) {
			objectStates_.add(new OOMDPObjectState(objState));
		}
		
		Collections.sort(objectStates_); // Important!
		
		relations_ = new Vector<Relation>();
		relations_.addAll(Arrays.asList(stateMessage.relations));
	}
	
	public OOMDPState(List<OOMDPObjectState> objectStates, List<Relation> relations) {
		objectStates_ = objectStates;
		
		Collections.sort(objectStates_); // Important!
		
		relations_ = relations;
	}
	
	public List<OOMDPObjectState> getObjectStates() {
		return objectStates_;
	}
	
	public MDPState convertToROS() {
		MDPState state = new MDPState();
		
		state.object_states = Collections2.transform(objectStates_, new Function<OOMDPObjectState, MDPObjectState>() {
			public MDPObjectState apply(OOMDPObjectState state) { return state.convertToROS(); }
		}).toArray(new MDPObjectState[0]);
		
		state.relations = relations_.toArray(new Relation[0]);
		
		return state;
	}
	
	public Set<String> getActiveRelations() {
		Set<String> result = new HashSet<String>();
		
		for (Relation rel : relations_) {
			if (rel.value > 0) { // It's actually boolean
				result.add(StateUtils.formatRelation(rel));
			}
		}
		
		return result;
	}
	
	@Override
	public String toString() {
		if (hashString_ == null) {
			hashString_ = new String();
			
			for (OOMDPObjectState os : objectStates_) {
				hashString_ += os.toString();
			}
			
			for (Relation rel : relations_) {
				if (rel.value > 0) {
					hashString_ += StateUtils.formatRelation(rel);
				}
			}
		} 
		
		return hashString_;
	}
}
