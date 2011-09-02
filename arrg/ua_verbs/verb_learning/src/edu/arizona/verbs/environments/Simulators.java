package edu.arizona.verbs.environments;

import edu.arizona.verbs.environments.gazebo.GazeboEnvironment;
import edu.arizona.verbs.environments.ww2d.WW2DEnvironment;
import edu.arizona.verbs.shared.Environment;

public enum Simulators {
	gazebo {
		@Override
		public Environment create() {
			return new GazeboEnvironment();
		}
	},
	
	ww2d {
		@Override
		public Environment create() {
			return new WW2DEnvironment(WW2DEnvironment.visualize);
		}
	};
	
	public abstract Environment create();
}
