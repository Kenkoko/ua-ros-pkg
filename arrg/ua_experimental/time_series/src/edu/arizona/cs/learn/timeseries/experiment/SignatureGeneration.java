package edu.arizona.cs.learn.timeseries.experiment;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Random;

import org.apache.log4j.Logger;

import edu.arizona.cs.learn.timeseries.model.Instance;
import edu.arizona.cs.learn.timeseries.model.SequenceType;
import edu.arizona.cs.learn.timeseries.model.Signature;
import edu.arizona.cs.learn.util.Utils;

public class SignatureGeneration {
    private static Logger logger = Logger.getLogger(SignatureGeneration.class);

	public static void main(String[] args) { 
		Utils.LIMIT_RELATIONS = true;
		Utils.WINDOW = 5;
		signature();
//		sequence();
	}
	
	public static void signature() { 
		String name = "ww2d-fight";
		String file = "data/input/" + name + ".lisp";

    	List<Instance> instances = Utils.sequences(name, file, SequenceType.allen);

    	// now construct a copy and randomly shuffle it.
    	Random r = new Random(System.currentTimeMillis());
    	List<Instance> random = new ArrayList<Instance>(instances);
    	Collections.shuffle(random, r);
    	
    	Signature s1 = new Signature(name);
		logger.debug("Training signature on " + random.size() + " sequences");
    	for (Instance instance : random) { 
    		logger.debug("  Instance " + instance.id());
    		s1.update(instance.sequence());
    	}
    	s1 = s1.prune(s1.trainingSize() / 2);
    	s1.toXML("/tmp/" + name + "-signature.xml");

//    	for (String method : new String[] { "single", "complete", "average" }) { 
//        	Signature s2 = Signature.agglomerativeTraining(method, instances);
//        	s2 = s2.prune(s2.trainingSize() / 2);
//        	s2.toXML("/Users/wkerr/Desktop/signature-" + method + ".xml");
//    	}

//    	generator = SequenceFactory.forwardAllenGenerator();
//    	instances = generator.sequences(name, "data/input/" + name + ".lisp");
//    	s = new Signature(name);
//    	s.train(instances);
//    	s.toFile("/tmp/signature2.signature", s.trainingSize()/2);
	}
	
	/**
	 * I just want to generate a sequence to see how large it is.
	 */
	public static void sequence() { 
		for (int i = 1; i <= 5; ++i) { 
			String name = "ww2d-test-" + i;
			List<Instance> instances = Utils.sequences(name, "data/input/" + name + ".lisp", SequenceType.allen);
			logger.debug("Sequence Size: " + instances.get(0).sequence().size());
		}
	}
}
