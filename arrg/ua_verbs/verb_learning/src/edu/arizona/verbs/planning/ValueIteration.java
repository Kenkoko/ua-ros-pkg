package edu.arizona.verbs.planning;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import edu.arizona.verbs.shared.Environment;
import edu.arizona.verbs.shared.OOMDPState;
import edu.arizona.verbs.verb.irl.IRLVerb;

public class ValueIteration {
	protected int numStates;
	protected int numActions;
	protected double [] vals;
	protected double gamma;
	protected HashMap<String, Integer> trans = new HashMap<String, Integer>();
	protected List<Double> rew = new ArrayList<Double>();;
	
	Environment myEnv;
	IRLVerb myVerb;
	
	public ValueIteration(Environment e, IRLVerb v, double g){
		numStates = 0;
		numActions = e.getActions().size();
		gamma = g;
		myEnv = e;
		myVerb = v;
	}
	
	protected HashMap<String, Integer> stateMap;
	
	public void populate(OOMDPState s0, int hor){
		//do bfs
		stateMap = new HashMap<String, Integer>();
		List<OOMDPState> theQ = new ArrayList<OOMDPState>();
		List<Integer> depths = new ArrayList<Integer>();
		stateMap.put(s0.toString(), 0);
		theQ.add(s0);
		depths.add(0);
		rew.add(myVerb.getReward(s0));
		while(theQ.size() > 0){
		//update numStates
			OOMDPState s = theQ.remove(0);
			int d = depths.remove(0).intValue();
			for(int a = 0; a < numActions; a++){
				OOMDPState nextState = myEnv.simulateAction(s,myEnv.getActions().get(a));
				if(!stateMap.containsKey(nextState.toString()) && d+1 <= hor){
					stateMap.put(nextState.toString(), stateMap.keySet().size());
					theQ.add(nextState);
					depths.add(d + 1);
					rew.add(myVerb.getReward(nextState));
				}
				
				if (d+1 > hor && stateMap.get(nextState.toString()) == null) {
					trans.put( stateMap.get(s.toString()) + "!!!" + a, stateMap.get(s.toString()));
				} else {
					trans.put( stateMap.get(s.toString()) + "!!!" + a, stateMap.get(nextState.toString()));
				}
			}
			
		}
		numStates = stateMap.keySet().size();
		vals = new double[numStates];
		for(int i =0; i < vals.length; i++)
			vals[i] = 0.0;
	}
	
	protected int[] pol;
	
	public String getAction(OOMDPState s){
		// TODO Why are the NPEs happening here? does it have to do with the horizon?
		return myEnv.getActions().get(pol[stateMap.get(s.toString())]);
	}
	
	public int[] iterate(double epsilon, int maxIts){
		int it = 0;
		double delta = Double.MAX_VALUE;
		pol = new int[numStates];
		while(it < maxIts && delta > epsilon){
			it++;
			delta = 0.0;
			for(int s = 0; s < numStates; s++){
				double [] q =new double[numActions];
				int maxAct = 0; 
				for(int a = 0; a < numActions; a++){
					Integer index = trans.get(s + "!!!" + a);
					q[a] = rew.get(s) +  gamma * vals[index];
					if(q[a] > q[maxAct])
						maxAct = a;
				}
				delta = Math.max(delta,Math.abs(q[maxAct] - vals[s]) );
				vals[s] = q[maxAct];
				pol[s] = maxAct;
			}
			//System.out.println("Iteration " + it);
			//printVals();
		}
		return pol;
	}
	
	
	
	protected void printVals(){
		for(int i = 0; i < numStates; i++)
			System.out.println(i + " val: " + vals[i]);
		System.out.print("\n\n");
	}
	/*
	public static void main(String [] args){
		
		int ns = 5;
		int na = 2;
		double right = 1.0;
		double noise = 1.0-right;
		double [][] r = {{0,0}, {0,0}, {0,0}, {0,0}, {0,1}};
		double [][][] t = new double[ns][na][ns];
		for(int s = 0; s < ns; s++){
			for(int a= 0; a < na; a++){
				for(int sp = 0; sp < ns; sp++){
					if(sp == s +1 && a == 1 || s == sp && s== 4 && a ==1)
						t[s][a][sp] = right;
					else if(sp == s +1 && a == 0 || s == sp && s== 4 && a ==0)
						t[s][a][sp] = noise;
					else if(sp == s -1 && a == 0 || s == sp && s== 0 && a ==0)
						t[s][a][sp] = right;
					else if(sp == s -1 && a == 1 || s == sp && s== 0 && a ==1)
						t[s][a][sp] = noise;
					else
						t[s][a][sp] = 0.0;
				}
			}
		}
		
		VIExample vi = new VIExample(5,2,0.9, t, r);
		vi.iterate(0.00001, Integer.MAX_VALUE);
	}*/
	
}
