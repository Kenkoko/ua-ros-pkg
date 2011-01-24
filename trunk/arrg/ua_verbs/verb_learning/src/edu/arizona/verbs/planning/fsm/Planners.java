package edu.arizona.verbs.planning.fsm;

import edu.arizona.simulator.ww2d.external.WW2DEnvironment;
import edu.arizona.verbs.environments.GazeboEnvironment;
import edu.arizona.verbs.main.Interface;
import edu.arizona.verbs.planning.shared.Planner;
import edu.arizona.verbs.verb.vfsm.FSMVerb;

public class Planners {
	public static Planner getPlanner(FSMVerb verb, int maxDepth) {
		if (Interface.getCurrentEnvironment() instanceof GazeboEnvironment) {
			return new SearchPlanner(verb, Interface.getCurrentEnvironment());

		} else if (Interface.getCurrentEnvironment() instanceof WW2DEnvironment) {
			return new UCT(verb, Interface.getCurrentEnvironment(), maxDepth);
		
		} else {
			throw new RuntimeException("WHAT ENVIRONMENT ARE YOU USING?");
			// Sorry LRTDP...
			//planner = new LRTDP(this, Interface.getCurrentEnvironment());
		}
	}
}
