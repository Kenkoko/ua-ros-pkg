package edu.arizona.planning.mdp;

import java.util.Collections;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.Vector;

public class OOMDPState {

	private List<OOMDPObjectState> objectStates_;
	private List<Relation> relations_;
	private String hashString_ = null;
	
	public OOMDPState(List<OOMDPObjectState> objectStates, List<Relation> relations) {
		objectStates_ = objectStates;
		
		Collections.sort(objectStates_); // Important!
		
		relations_ = relations;
	}
	
	public List<OOMDPObjectState> getObjectStates() {
		return objectStates_;
	}
	
	public OOMDPState remapState(Map<String,String> nameMap) {
		Vector<OOMDPObjectState> newObjects = new Vector<OOMDPObjectState>();
		for (OOMDPObjectState obj : objectStates_) {
			newObjects.add(obj.remapState(nameMap));
		}
		Vector<Relation> newRelations = new Vector<Relation>();
		for (Relation rel : relations_) {
			Relation newRel = new Relation();
			newRel.relation = rel.relation;
			newRel.value = rel.value;
			for (int i = 0; i < rel.objectNames.size(); i++) {
				newRel.objectNames.add((nameMap.containsKey(rel.objectNames.get(i)) ? nameMap.get(rel.objectNames.get(i)) : rel.objectNames.get(i))); 
			}
			newRelations.add(newRel);
		}
		
		OOMDPState newState = new OOMDPState(newObjects, newRelations);
		return newState;
	}
	
	public Set<String> getActiveRelations() {
		Set<String> result = new HashSet<String>();
		
		for (Relation rel : relations_) {
			if (rel.value) { // It's actually boolean
				result.add(rel.toString());
			}
		}
		
		return result;
	}
	
	public List<Relation> getRelations() {
		return relations_;
	}
	
	@Override
	public String toString() {
		if (hashString_ == null) {
			hashString_ = new String();
			
			for (OOMDPObjectState os : objectStates_) {
				hashString_ += os.toString();
			}
			
			for (Relation rel : relations_) {
				if (rel.value) {
					hashString_ += rel;
				}
			}
		} 
		
		return hashString_;
	}
}
