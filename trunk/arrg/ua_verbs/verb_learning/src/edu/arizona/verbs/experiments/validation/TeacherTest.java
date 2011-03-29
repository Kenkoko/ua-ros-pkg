package edu.arizona.verbs.experiments.validation;

import java.util.List;

import edu.arizona.simulator.ww2d.external.WW2DEnvironment;
import edu.arizona.verbs.environments.Simulators;
import edu.arizona.verbs.experiments.Experimenter;
import edu.arizona.verbs.experiments.Scenario;
import edu.arizona.verbs.experiments.evaluation.Label;
import edu.arizona.verbs.experiments.evaluation.Labeler;
import edu.arizona.verbs.shared.Environment;
import edu.arizona.verbs.shared.OOMDPState;

public class TeacherTest {

	/**
	 * @param args
	 * @throws Exception 
	 */
	public static void main(String[] args) throws Exception {
		String env = "ww2d";
		String verb = "follow";
		int index = 9;
		
		List<Scenario> scenarios = Experimenter.loadScenarios(env, verb);

		WW2DEnvironment.visualize = true;
		Environment environment = Simulators.valueOf(env).create();
		
		Scenario scenario = scenarios.get(index);
		
		System.out.println(scenario.start);
		
		List<OOMDPState> demonstration = Experimenter.demonstrate(scenario, environment);
		for (OOMDPState state : demonstration) {
			System.out.println(state);
		}
		Label label = Labeler.valueOf(verb).label(demonstration);
		System.out.println("LABEL:" + label);
	}
}
