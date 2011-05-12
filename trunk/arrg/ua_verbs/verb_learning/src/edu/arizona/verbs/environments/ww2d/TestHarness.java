package edu.arizona.verbs.environments.ww2d;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import edu.arizona.verbs.shared.OOMDPObjectState;
import edu.arizona.verbs.shared.OOMDPState;
import edu.arizona.verbs.shared.Relation;

public class TestHarness {
	
	public static void test1Agent() throws SimulatorFailureException { 
		// Create simple initial environment. Single agent as a circle.
		OOMDPObjectState obj1 = new OOMDPObjectState("agent1", "agent");
		obj1.setAttribute("x", "10");
		obj1.setAttribute("y", "10");
		obj1.setAttribute("angle", "0.00");
		obj1.setAttribute("shape-type", "circle");
		obj1.setAttribute("radius", "0.25");

		List<OOMDPObjectState> objects = new ArrayList<OOMDPObjectState>();
		objects.add(obj1);
		
		WW2DEnvironment env = new WW2DEnvironment(true);
		OOMDPState state = env.initializeEnvironment(objects);

		System.out.println("Initialization complete...");
		System.out.println(state.toString());
		
		System.out.println("Printing actions...");
		System.out.println(env.getActions());
		
		System.out.println(env.performAction("agent1 1000"));
		System.out.println(env.performAction("agent1 1000"));
		System.out.println(env.performAction("agent1 1000"));
		System.out.println(env.performAction("agent1 1000"));
		
		env.cleanup();
	}

	public static void testAgentEnemy() throws SimulatorFailureException {
		OOMDPObjectState person = new OOMDPObjectState("person", "agent");
		person.setAttribute("x", "60");
		person.setAttribute("y", "55");
		person.setAttribute("heading", "-3");
		person.setAttribute("vx", "0");
		person.setAttribute("vy", "0");
		person.setAttribute("vtheta", "0");
		person.setAttribute("shape-type", "circle");
		person.setAttribute("radius", "0.25");
		
		OOMDPObjectState enemy = new OOMDPObjectState("enemy", "agent");
		enemy.setAttribute("x", "40");
		enemy.setAttribute("y", "50");
		enemy.setAttribute("heading", "0");
		enemy.setAttribute("vx", "0");
		enemy.setAttribute("vy", "0");
		enemy.setAttribute("vtheta", "0");
		enemy.setAttribute("shape-type", "circle");
		enemy.setAttribute("radius", "0.25");
		
		OOMDPObjectState place = new OOMDPObjectState("place", "obstacle");
		place.setAttribute("x", "70");
		place.setAttribute("y", "50");
		place.setAttribute("heading", "0");
		place.setAttribute("vx", "0");
		place.setAttribute("vy", "0");
		place.setAttribute("vtheta", "0");
		place.setAttribute("shape-type", "circle");
		place.setAttribute("radius", "0.5");
		
		List<OOMDPObjectState> objects = new ArrayList<OOMDPObjectState>();
		objects.add(person);
		objects.add(enemy);
		objects.add(place);
		
		WW2DEnvironment env = new WW2DEnvironment(true);
		OOMDPState state = env.initializeEnvironment(objects);

		System.out.println("Initialization complete...");
		System.out.println(state.toString());
		
		System.out.println("Printing actions...");
		List<String> actions = env.getActions();
		System.out.println(actions.size() + " actions...");
		System.out.println(env.getActions());

		for (int i = 0; i < 50; i++) {
			if (i < 2) {
				state = env.performAction("person 1000");
			} else {
				state = env.performAction("person 0000");
			}
			System.out.println(state.getActiveRelations());
		}
	}
	
	public static void testAgentObstacle() throws SimulatorFailureException { 
		OOMDPObjectState obj1 = new OOMDPObjectState("person", "agent");
		obj1.setAttribute("x", "50");
		obj1.setAttribute("y", "50");
		obj1.setAttribute("heading", "0");
		obj1.setAttribute("vx", "0");
		obj1.setAttribute("vy", "0");
		obj1.setAttribute("vtheta", "0");
		obj1.setAttribute("shape-type", "circle");
		obj1.setAttribute("radius", "0.25");

//		OOMDPObjectState obj2 = new OOMDPObjectState("agent2", "agent");
//		obj2.setAttribute("x", "20");
//		obj2.setAttribute("y", "10");
//		obj2.setAttribute("heading", "3.14");
//		obj2.setAttribute("shape-type", "circle");
//		obj2.setAttribute("radius", "0.25");
		
		OOMDPObjectState obj2 = new OOMDPObjectState("place", "obstacle");
		obj2.setAttribute("x", "55");
		obj2.setAttribute("y", "45");
		obj2.setAttribute("heading", "0");
		obj2.setAttribute("vx", "0");
		obj2.setAttribute("vy", "0");
		obj2.setAttribute("vtheta", "0");
		obj2.setAttribute("shape-type", "circle");
		obj2.setAttribute("radius", "0.5");
		
		List<OOMDPObjectState> objects = new ArrayList<OOMDPObjectState>();
		objects.add(obj1);
		objects.add(obj2);
		
		WW2DEnvironment env = new WW2DEnvironment(true);
		OOMDPState state = env.initializeEnvironment(objects);

//		System.out.println("Initialization complete...");
//		System.out.println(state.toString());
//		
//		System.out.println("Printing actions...");
//		List<String> actions = env.getActions();
//		System.out.println(actions.size() + " actions...");
//		System.out.println(env.getActions());

		List<String> actions = env.getActions();
		Random r = new Random();
		
		
		System.out.println("BEGIN PERFORM");
		System.out.println(state);
//		for (int i = 0; i < 12; i++) {
			 //;agent2 0000");
//			OOMDPState newState = env.performAction(actions.get(r.nextInt(actions.size())));
//			System.out.println(newState);
//		}
		// Forward Left Right Back
		OOMDPState newState = env.performAction("person 0100");
		newState = env.performAction("person 0100");
//		newState = env.performAction("person 0100");
//		newState = env.performAction("person 0100");
//		newState = env.performAction("person 0100");
//		newState = env.performAction("person 0100");
		
		newState = env.performAction("person 1000");
//		newState = env.performAction("person 0100");
		newState = env.performAction("person 1000");
		newState = env.performAction("person 1000");
		newState = env.performAction("person 1000");
		

		System.out.println("FINAL: " + newState);
		
		
//		System.out.println("BEGIN SIMULATE");
//		OOMDPState simState = env.initializeEnvironment(objects);
//		System.out.println(simState);
//		for (int i = 0; i < 12; i++) {
//			System.out.println("================ LOOP " + i);
//			simState = env.simulateAction(simState, "agent1 1000");
//			simState = env.simulateAction("agent1 1000");
//			System.out.println("TEST 1: " + simState);
//			env.simulateAction(simState, "agent1 0100");
//			System.out.println("TEST 2: " + simState);
//			System.out.println(simState);
//		}
		
		env.cleanup();
	}
	
	public static void test3Agent() throws SimulatorFailureException { 
		OOMDPObjectState obj1 = new OOMDPObjectState("agent1", "agent");
		obj1.setAttribute("x", "10");
		obj1.setAttribute("y", "10");
		obj1.setAttribute("heading", "0.00");
		obj1.setAttribute("shape-type", "circle");
		obj1.setAttribute("radius", "0.25");

		OOMDPObjectState obj2 = new OOMDPObjectState("agent2", "agent");
		obj2.setAttribute("x", "20");
		obj2.setAttribute("y", "10");
		obj2.setAttribute("heading", "3.14");
		obj2.setAttribute("shape-type", "circle");
		obj2.setAttribute("radius", "0.25");
		
		List<OOMDPObjectState> objects = new ArrayList<OOMDPObjectState>();
		objects.add(obj1);
		objects.add(obj2);
		
		WW2DEnvironment env = new WW2DEnvironment(true);
		OOMDPState state = env.initializeEnvironment(objects);

		System.out.println("Initialization complete...");
		System.out.println(state.toString());
		
		System.out.println("Printing actions...");
		List<String> actions = env.getActions();
		System.out.println(actions.size() + " actions...");
		System.out.println(env.getActions());

		System.out.println(env.performAction("agent1 1000;agent2 1000"));
		System.out.println(env.performAction("agent1 1000;agent2 1000"));
		System.out.println(env.performAction("agent1 1000;agent2 1000"));
		System.out.println(env.performAction("agent1 1000;agent2 1000"));
		
		obj1.setAttribute("vx", "-30.00");
		obj1.setAttribute("vy", "0.00");
		obj1.setAttribute("vtheta", "0.000");

		obj2.setAttribute("vx", "1.00");
		obj2.setAttribute("vy", "0.00");
		obj2.setAttribute("vtheta", "0.000");

		System.out.println(env.simulateAction(new OOMDPState(objects, new ArrayList<Relation>()), "agent1 1000;agent2 1000"));
		
		env.cleanup();
	}
	
	
	public static void main(String[] args) throws SimulatorFailureException { 
//		test1Agent();
//		testAgentObstacle();
		testAgentEnemy();
//		test3Agent();
	}
}
