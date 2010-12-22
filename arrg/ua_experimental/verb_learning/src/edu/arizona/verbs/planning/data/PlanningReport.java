package edu.arizona.verbs.planning.data;

import edu.arizona.verbs.planning.shared.Policy;

public class PlanningReport {
	private boolean success_;
	private Policy policy_;
	private long elapsedTime_;
	
	public PlanningReport(Policy policy, boolean success, long time) {
		policy_ = policy;
		success_ = success;
		elapsedTime_ = time;
	}

	public boolean wasSuccessful() {
		return success_;
	}

	public Policy getPolicy() {
		return policy_;
	}

	public long getElaspedTime() {
		return elapsedTime_;
	}
}
