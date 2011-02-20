package edu.arizona.verbs.loading;

import java.util.ArrayList;
import java.util.List;

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
	
	public OOMDPState convert() {
		OOMDPState oomdpState = new OOMDPState(new ArrayList<OOMDPObjectState>(), relations);
		return oomdpState;
	}
}
