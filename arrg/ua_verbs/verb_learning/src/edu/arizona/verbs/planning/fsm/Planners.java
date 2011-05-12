package edu.arizona.verbs.planning.fsm;

import edu.arizona.verbs.environments.gazebo.GazeboEnvironment;
import edu.arizona.verbs.environments.ww2d.WW2DEnvironment;
import edu.arizona.verbs.planning.shared.Planner;
import edu.arizona.verbs.shared.Environment;
import edu.arizona.verbs.verb.vfsm.FSMVerb;

public class Planners {
	public static Planner getPlanner(Environment env, FSMVerb verb, int maxDepth) {
		if (env instanceof GazeboEnvironment) {
			return new SearchPlanner(verb, env);

		} else if (env instanceof WW2DEnvironment) {
			return new UCT(verb, env, maxDepth);
		
		} else {
			throw new RuntimeException("WHAT ENVIRONMENT ARE YOU USING?");
			// Sorry LRTDP...
			//planner = new LRTDP(this, Interface.getCurrentEnvironment());
		}
	}
}
