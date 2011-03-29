package edu.arizona.verbs.loading;

import java.io.FileInputStream;
import java.io.FileNotFoundException;

import org.yaml.snakeyaml.Yaml;
import org.yaml.snakeyaml.constructor.Constructor;

import edu.arizona.verbs.shared.Relation;
import edu.arizona.verbs.verb.vfsm.AtomicVerb;
import edu.arizona.verbs.verb.vfsm.learner.CorePathLearner;
import edu.arizona.verbs.verb.vfsm.learner.LearnerType;

public class LoadingTest {
	
	/**
	 * @param args
	 * @throws FileNotFoundException 
	 */
	public static void main(String[] args) throws FileNotFoundException {
		FileInputStream fis = new FileInputStream("scanBoat.yaml");
//		FileInputStream fis = new FileInputStream("example.yaml");
		
		Constructor constructor = new Constructor(VerbData.class);
		Yaml yaml = new Yaml(constructor);
		
		VerbData vd = (VerbData) yaml.load(fis);
		
		System.out.println(vd.getVerb());
		System.out.println(vd.getArguments());
		
		AtomicVerb verb = new AtomicVerb(vd.getVerb(), vd.getArguments(), LearnerType.corepath);
		
		for (Trace t : vd.getTraces()) {
//			for (SimpleRelationalState s : t.getStates()) {
//				System.out.println("STATE:");
//				for (Relation r : s.getRelations()) {
//					System.out.println("\t" + r + "=" + r.getValue());
//					System.out.println(r.getRelation());
//					System.out.println(r.getArguments());
//					System.out.println(r.getValue());
//				}
//			}
			
			if (t.getLabel().equals("positive")) {
				verb.addPositiveInstance(t.convert());
			} else {
				verb.addNegativeInstance(t.convert());
			}
		}
		
		// Why is there a problem with minimize for this data?
//		verb.getPositiveFSM().minimize();
//		verb.getPositiveFSM().toDot("min.dot", false);
	}

}
