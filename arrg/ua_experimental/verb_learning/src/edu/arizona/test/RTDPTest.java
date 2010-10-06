package edu.arizona.test;

import java.util.Vector;

import ros.pkg.simulator_state.msg.SimpleRelation;

import edu.arizona.cs.learn.timeseries.model.Signature;
import edu.arizona.environment.Environment;
import edu.arizona.planning.RTDP;
import edu.arizona.planning.mdp.LocationState;
import edu.arizona.planning.mdp.MDPObjectState;
import edu.arizona.planning.mdp.MDPState;
import edu.arizona.planning.mdp.RobotState;
import edu.arizona.verbs.Verb;

public class RTDPTest {
	/**
	 * @param args
	 */
	public static void main(String[] args) {
		String signatureFile = "verbs/go/signature.xml";
        Verb go = new Verb("go", Signature.fromXML(signatureFile));
        
        if (true) { return; }
        
        Environment env = new Environment();
        
        Vector<MDPObjectState> objectStartStates = new Vector<MDPObjectState>();
        MDPObjectState robot = new RobotState("thing", 0, 0, "E", 0, 0, "E");
        objectStartStates.add(robot);
        MDPObjectState goal = new LocationState("place", 3.0, 2.0);
        objectStartStates.add(goal);
        
        Vector<SimpleRelation> relations = new Vector<SimpleRelation>();
        SimpleRelation rel = new SimpleRelation();
        rel.rel_name = "InFrontOf";
        rel.obj_names = new String[] {"thing", "place"};
        rel.value = 1;
        relations.add(rel);
        MDPState start = new MDPState(objectStartStates, relations);
        
        System.out.println(start);
        
        long startTime = System.currentTimeMillis();
        RTDP rtdp = new RTDP(go, env, start);
        rtdp.runAlgorithm();
        System.out.println("DURATION: " + ((System.currentTimeMillis() - startTime)/1000) + " seconds");
        rtdp.recoverPolicy();
	}
}
