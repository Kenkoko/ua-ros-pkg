package edu.arizona.planning.fsm;

import java.util.Set;
import java.util.TreeSet;

import edu.arizona.cs.learn.util.Utils;

public class FSMTransition {
private Set<String> _props;
	
	private String _label;
	private double _count;

	public FSMTransition(Set<String> props) {
		_props = new TreeSet<String>(props);

		StringBuffer buf = new StringBuffer();
		StringBuffer key = new StringBuffer("[");
		for (String s : this._props) {
			if (buf.length() > 0) {
				buf.append("\\n");
			}
			if (key.length() > 1)
				key.append(" ");
			buf.append(s);
			key.append(s);
		}
		_label = buf.toString();

		_count = 0.0D;
	}

	@Override
	public boolean equals(Object obj) {
		if (obj instanceof FSMTransition) {
			return this.props().equals(((FSMTransition) obj).props());
		} else {
			return false;
		}
	}

	@Override
	public int hashCode() {
		return _props.hashCode();
	}

	@Override
	public String toString() {
		return _props.toString();
	}

	public Set<String> props() {
		return _props;
	}

	public void increment() {
		_count += 1.0D;
	}

	public void increment(double count) {
		_count += count;
	}

	public double count() {
		return _count;
	}

	public boolean satisfied(Set<String> props) {
		return props.containsAll(_props);
	}

	public String toDot(boolean edgeProb, double prob) {
		if (!edgeProb) {
			return " [label=\"" + this._label + "\"] ";
		}
		return " [label=\"" + this._label + "\\n" + Utils.nf.format(prob) + "\"] ";
	}
}
