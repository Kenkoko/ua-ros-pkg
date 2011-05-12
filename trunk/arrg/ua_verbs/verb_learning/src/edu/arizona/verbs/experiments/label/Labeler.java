package edu.arizona.verbs.experiments.label;

import java.util.List;

import edu.arizona.verbs.shared.OOMDPState;

public interface Labeler {
	public Label label(List<OOMDPState> trace);
}
