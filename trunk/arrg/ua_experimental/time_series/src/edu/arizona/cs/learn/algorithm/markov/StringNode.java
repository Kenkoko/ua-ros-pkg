package edu.arizona.cs.learn.algorithm.markov;

import java.util.Set;

public class StringNode extends BPPNode {

	private String _label;

	public StringNode(String label, BPPNode startNode) {
		super(startNode);

		_label = label;
	}

	public String label() {
		return _label;
	}

	public boolean active(Set<String> activeProps) {
		return true;
	}
}