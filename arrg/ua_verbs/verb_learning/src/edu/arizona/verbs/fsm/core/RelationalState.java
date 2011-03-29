package edu.arizona.verbs.fsm.core;

import java.util.Collection;
import java.util.List;
import java.util.TreeSet;

import com.google.common.collect.Lists;

import edu.arizona.verbs.shared.Relation;

public class RelationalState {

	private TreeSet<Relation> relations_ = new TreeSet<Relation>();
	
	private List<Relation> allRelations_;
	
	public RelationalState(Collection<Relation> relations) {
		for (Relation r : relations) {
			if (r.getValue()) {
				relations_.add(r);
			}
		}
		
		allRelations_ = Lists.newArrayList(relations);
	}

	public TreeSet<Relation> getRelations() {
		return relations_;
	}
	
	public List<Relation> getAllRelations() {
		return allRelations_;
	}
	
	public boolean isSuperset(RelationalState other) {
		return relations_.containsAll(other.relations_);
	}
	
	@Override
	public String toString() {
		return relations_.toString();
	}
	
	@Override
	public int hashCode() {
		return relations_.hashCode();
	}

	@Override
	public boolean equals(Object obj) {
		return (obj instanceof RelationalState &&
				relations_.equals(((RelationalState) obj).relations_));
	}
}
