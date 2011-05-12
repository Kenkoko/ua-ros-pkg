package edu.arizona.verbs.experiments.validation;

import java.util.List;

import edu.arizona.verbs.environments.Simulators;
import edu.arizona.verbs.environments.ww2d.WW2DEnvironment;
import edu.arizona.verbs.experiments.Experimenter;
import edu.arizona.verbs.experiments.Scenario;
import edu.arizona.verbs.experiments.label.Label;
import edu.arizona.verbs.experiments.label.Labelers;
import edu.arizona.verbs.shared.Environment;
import edu.arizona.verbs.shared.OOMDPState;

public class TeacherTest {

	/**
	 * @param args
	 * @throws Exception 
	 */
	public static void main(String[] args) throws Exception {
		String env = "ww2d";
//		String env = "gazebo";
		String verb = "intercept";
		int index = 3;
		
		List<Scenario> scenarios = Experimenter.loadScenarios(env, verb);

		WW2DEnvironment.visualize = true;
		Environment environment = Simulators.valueOf(env).create();
		
		Scenario scenario = scenarios.get(index);
		
		System.out.println(scenario.start);
		
		List<OOMDPState> demonstration = Experimenter.demonstrate(scenario, environment);
		for (OOMDPState state : demonstration) {
			System.out.println(state);
		}
		Label label = Labelers.valueOf(env).getLabeler(verb).label(demonstration);
		System.out.println("LABEL:" + label);
	}
}
