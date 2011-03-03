package edu.arizona.verbs.loading;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import edu.arizona.verbs.shared.OOMDPObjectState;
import edu.arizona.verbs.shared.OOMDPState;
import edu.arizona.verbs.shared.Relation;

public class SimpleRelationalState {
	private List<Relation> relations;

	public List<Relation> getRelations() {
		return relations;
	}

	public void setRelations(List<Relation> relations) {
		this.relations = relations;
	}
	
	public SimpleRelationalState remap(Map<String, String> bindings) {
		ArrayList<Relation> newRelations = new ArrayList<Relation>();
		
		for (Relation rel : relations) {
			newRelations.add(rel.remap(bindings));
		}
		
		SimpleRelationalState newState = new SimpleRelationalState();
		newState.setRelations(newRelations);
		return newState;
	}
	
	public OOMDPState convert(Map<String, String> bindings) {
		SimpleRelationalState state = this.remap(bindings);
		OOMDPState oomdpState = new OOMDPState(new ArrayList<OOMDPObjectState>(), state.relations);
		return oomdpState;
	}
}
