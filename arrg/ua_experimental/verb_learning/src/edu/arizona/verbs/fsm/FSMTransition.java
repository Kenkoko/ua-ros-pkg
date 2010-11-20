package edu.arizona.verbs.fsm;

import java.util.Collection;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.TreeSet;
import java.util.Vector;

import com.google.common.base.Splitter;
import com.google.common.collect.Iterables;

import edu.arizona.cs.learn.util.Utils;
import edu.arizona.verbs.shared.Relation;
import edu.arizona.verbs.shared.Remappable;

public class FSMTransition implements Remappable<FSMTransition> {
	private Set<Relation> relations_ = new HashSet<Relation>(); // Is there a reason to use TreeSet?
	private Set<String> props_;
	private String label_;
	private double count_;

	// Collection instead of set to avoid generic type erasure
	public FSMTransition(Collection<Relation> relations) {
		props_ = new TreeSet<String>();
		relations_ = new HashSet<Relation>(relations);
		
		StringBuffer buf = new StringBuffer();
		for (Relation r : relations) {
			props_.add(r.toString());
			if (buf.length() > 0) {
				buf.append("\\n");
			}
			buf.append(r.toString());
		}
		label_ = buf.toString();

		count_ = 0.0D;
	}
	
	public FSMTransition(Set<String> props) {
		props_ = new TreeSet<String>(props);
		
		StringBuffer buf = new StringBuffer();
		for (String s : this.props_) {
			relations_.add(parsePropString(s));
			if (buf.length() > 0) {
				buf.append("\\n");
			}
			buf.append(s);
		}
		label_ = buf.toString();

		count_ = 0.0D;
	}

	@Override
	public String toString() {
		return props_.toString();
	}

	public void increment() {
		count_ += 1.0D;
	}

	public void increment(double count) {
		count_ += count;
	}

	public double count() {
		return count_;
	}

	public String toDot(boolean edgeProb, double prob) {
		if (!edgeProb) {
			return " [label=\"" + this.label_ + "\"] ";
		}
		return " [label=\"" + this.label_ + "\\n" + Utils.nf.format(prob) + "\"] ";
	}

	public Set<String> getSymbol() {
		return props_;
	}
	
	public Set<Relation> getActiveRelations() {
		return relations_;
	}

	public boolean accept(Set<String> props) {
		return props.containsAll(props_);
	}
	
	// This is a bit silly but it's the easiest solution for now
	private Relation parsePropString(String prop) {
		prop = prop.substring(0, prop.length() - 1); // Drop the trailing ')'
		Iterable<String> it = Splitter.on('(').split(prop); //
		Vector<String> tokens = new Vector<String>();
		Iterables.addAll(tokens, it);
		String relation = tokens.remove(0);
		List<String> objectNames = new Vector<String>();
		if (!tokens.isEmpty()) {
			Iterable<String> names = Splitter.on(',').split(tokens.remove(0));
			Iterables.addAll(objectNames, names);
		}
		return new Relation(relation, objectNames.toArray(new String[0]), true);
	}

	@Override
	public FSMTransition remap(Map<String, String> nameMap) {
		Set<Relation> newRelations = new HashSet<Relation>();
		
		for (Relation r : relations_) {
			newRelations.add(r.remap(nameMap));
		}
		
		return new FSMTransition(newRelations);
	}
}
