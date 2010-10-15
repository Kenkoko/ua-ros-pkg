package edu.arizona.test;

import java.util.HashMap;
import java.util.Vector;

import ros.pkg.oomdp_msgs.msg.Relation;

import edu.arizona.cs.learn.timeseries.model.Signature;
import edu.arizona.environment.Environment;
import edu.arizona.planning.RTDP;
import edu.arizona.planning.mdp.OOMDPObjectState;
import edu.arizona.planning.mdp.OOMDPState;
import edu.arizona.verbs.Verb;

public class RTDPTest {
	/**
	 * @param args
	 */
	public static void main(String[] args) {
		String signatureFile = "verbs/go/signature.xml";
        Verb go = new Verb("go", new String[]{"thing", "place"}, Signature.fromXML(signatureFile));
        
//        if (true) return;
        
        Environment env = new Environment();
        
        Vector<OOMDPObjectState> objectStartStates = new Vector<OOMDPObjectState>();
        OOMDPObjectState robot = new OOMDPObjectState("thing", "Robot");
        robot.setAttribute("x", "0.0");
        robot.setAttribute("y", "0.0");
        robot.setAttribute("orientation", "E");
        robot.setAttribute("last_x", "0.0");
        robot.setAttribute("last_y", "0.0");
        robot.setAttribute("last_orientation", "E");
        objectStartStates.add(robot);
        
        OOMDPObjectState goal = new OOMDPObjectState("place", "Location");
        goal.setAttribute("x", "-3.0"); // -3.0
        goal.setAttribute("y", "0.0");
        objectStartStates.add(goal);
        
        Vector<Relation> relations = new Vector<Relation>();
        Relation rel = new Relation();
        rel.relation = "InFrontOf";
        rel.obj_names = new String[] {"thing", "place"};
        rel.value = 1;
        relations.add(rel);
        OOMDPState start = new OOMDPState(objectStartStates, relations);
        
        System.out.println(start);
        
        long startTime = System.currentTimeMillis();
        RTDP rtdp = new RTDP(go, env, start);
        rtdp.runAlgorithm();
        System.out.println("DURATION: " + ((System.currentTimeMillis() - startTime)/1000) + " seconds");
        rtdp.recoverPolicy(new HashMap<String,String>());
	}
}
