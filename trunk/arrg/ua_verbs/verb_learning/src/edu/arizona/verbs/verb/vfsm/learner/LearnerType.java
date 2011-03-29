package edu.arizona.verbs.verb.vfsm.learner;


public enum LearnerType {
	signature {
		@Override
		public VFSMLearner create() {
			return new SignatureLearner(0.8); 
		}
	},
	
	corepath {
		@Override
		public VFSMLearner create() {
			return new CorePathLearner();
		}
	},
	
	naive {
		@Override
		public VFSMLearner create() {
			return new SignatureLearner(0.0); 
		}
	}
	
	;
	
	public abstract VFSMLearner create();
}
