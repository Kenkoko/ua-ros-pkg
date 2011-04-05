package edu.arizona.verbs.environments;

import edu.arizona.simulator.ww2d.external.WW2DEnvironment;
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
			return new WW2DEnvironment(true);
		}
	};
	
	public abstract Environment create();
}
