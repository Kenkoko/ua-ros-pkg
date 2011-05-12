package edu.arizona.verbs.experiments.label;

public enum Labelers {
	ww2d {
		@Override
		public Labeler getLabeler(String verb) {
			return WW2DLabeler.valueOf(verb);
		}
	}, 
	
	gazebo {
		@Override
		public Labeler getLabeler(String verb) {
			return GazeboLabeler.valueOf(verb);
		}
	}
	;
	
	public abstract Labeler getLabeler(String verb);
}
