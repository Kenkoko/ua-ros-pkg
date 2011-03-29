package edu.arizona.verbs.verb;

import java.util.ArrayList;

import edu.arizona.verbs.verb.bayes.MaximumLikelihoodVerb;
import edu.arizona.verbs.verb.irl.IRLVerb;
import edu.arizona.verbs.verb.vfsm.AtomicVerb;
import edu.arizona.verbs.verb.vfsm.learner.LearnerType;

public enum Verbs {
	signature {
		@Override
		public Verb create(String word, ArrayList<String> arguments) {
			return new AtomicVerb(word, arguments, LearnerType.signature); 
		}
	},
	
	corepath {
		@Override
		public Verb create(String word, ArrayList<String> arguments) {
			return new AtomicVerb(word, arguments, LearnerType.corepath); 
		}
	},
	
	naive {
		@Override
		public Verb create(String word, ArrayList<String> arguments) {
			return new AtomicVerb(word, arguments, LearnerType.naive); 
		}
	},
	
	ml {
		@Override
		public Verb create(String word, ArrayList<String> arguments) {
			return new MaximumLikelihoodVerb(word, arguments);
		}
	},
	
	irl {
		@Override
		public Verb create(String word, ArrayList<String> arguments) {
			return new IRLVerb(word, arguments);
		}
	};
	
	public abstract Verb create(String word, ArrayList<String> arguments);
}
