package edu.arizona.verbs.verb.irl;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import com.google.common.base.Joiner;
import com.google.common.collect.Lists;

import ros.pkg.oomdp_msgs.msg.MDPState;
import ros.pkg.verb_learning.srv.PerformVerb.Response;
import edu.arizona.verbs.main.Interface;
import edu.arizona.verbs.shared.Environment;
import edu.arizona.verbs.shared.OOMDPState;
import edu.arizona.verbs.shared.Relation;
import edu.arizona.verbs.verb.Verb;

import Jama.Matrix;  //this is in the zip file i sent you or download from nist.gov

public class IRLVerb implements Verb {

	private boolean teachingDone = false;
	private boolean convergenceAchieved = false;
	private ArrayList<List<OOMDPState>> teacherTraces = new ArrayList<List<OOMDPState>>();
	
	private String verb_;
	private ArrayList<String> arguments_;

	//IRL members
	protected double discount;
	protected double curDis;
	protected double [] weights;
	protected int samples;
	protected int samplesExp;
	protected Matrix mu;
	protected Matrix muExp;
	protected int hor;
	protected int dimensions;
	protected Matrix muBarOne;
	protected double epsilon;
	protected boolean learning;

	protected double dot(double[] a, double[]b){
		double tot = 0.0;
		for(int x = 0; x < a.length; x++)
			tot += a[x] * b[x];
		return tot;
	}

	// TODO: Have to fix this, arguments is the verb arguments not parameters
	public IRLVerb(String word, ArrayList<String> arguments) {
		verb_ = word;
		arguments_ = arguments;
		epsilon = Double.parseDouble(arguments.get(0));
		hor = Integer.parseInt(arguments.get(1));
		dimensions = Integer.parseInt(arguments.get(2));
		discount = Integer.parseInt(arguments.get(3));
		samples = 0;
		learning = true;
		weights = new double[dimensions];
		for(int x=0; x < dimensions; x++)
			weights[x] = 0.0;
		
	}
	
	@Override
	// Teacher traces will come in one at a time, to keep the code changes minimal
	// They will accumulate until...
	public void addPositiveInstance(List<OOMDPState> trace) {
		if (teachingDone) { // This means we are in the autonomous phase
			throw new RuntimeException("You said you were done teaching! You lied!");
		} else { // This means we are in the training phase 
			teacherTraces.add(trace);
		}
	}
	
	// ... the teacher calls this when they are done presenting instances
	public void teach() {
		List<int[][]> traces = new ArrayList<int[][]>();
		for (List<OOMDPState> trace : teacherTraces) {
			traces.add(traceToArray(trace));
		}
		samplesExp = traces.size();

		muExp = new Matrix(dimensions,1);
		for(int x=0; x < samplesExp; x++){
			double dis = 1.0;
			for(int st = 0; st < traces.get(x).length; st++){
				muExp =  muExp.plus(new Matrix(toDouble(traces.get(x)[st])).times(dis));
     		   	dis = discount * dis;
			}
		}
		muExp = muExp.times(1.0/((double)samplesExp));
		
		teachingDone = true;
	}

	
	protected double[][] toDouble(int [] s){
		double [][]d = new double[s.length][1];
		for(int x = 0; x < s.length; x++) {
			d[x][0] = (double)s[x];
		}
		return d;
	}
	
	@Override
	public void forgetInstances() {
		teachingDone = false;
		muExp = new Matrix(dimensions,1);
		weights = new double[dimensions];
		
		for(int x=0; x < dimensions; x++) {
			weights[x] = 0.0;
		}
	}

	// Return the action to do from the start state
	private void agentBegin(OOMDPState startState) {
		int[] s = stateToArray(startState);
		
		curDis = 1;
		samples += 1;
		mu =  new Matrix(toDouble(s));
	}
	
	private void agentStep(OOMDPState mdpState) {
		int[] s = stateToArray(mdpState);
		
  		curDis = discount * curDis;
     	mu = mu.plus(new Matrix(toDouble(s)).times(curDis));
	}
	
	private void agentEnd() {
		if(!learning)  //convergence was already reached!
			return;
		Matrix realMu = mu; //mu.times(1.0/((double)samples));
		Matrix muTilde;
		double [] oldWeights = weights;
    	if (samples == 1) {  //special case
      		muBarOne = realMu;
      		weights = muExp.minus(realMu).transpose().getArray()[0];
    	} else {
    		double num = realMu.minus(muBarOne).transpose().times(muExp.minus(muBarOne)).get(0,0);
    		double dom = realMu.minus(muBarOne).transpose().times(realMu.minus(muBarOne)).get(0,0);
    		muTilde = muBarOne.plus(realMu.minus(muBarOne).times(num/dom));

    		// I added the braces 
			if(checkConvex(muBarOne,mu,muTilde)) {
    			muBarOne = muTilde;
			} else if (muExp.minus(realMu).norm2() < muExp.minus(muBarOne).norm2()) {
       		 	muBarOne = realMu;
			} 
			//otherwise muBar stays
    		weights = muExp.minus(muBarOne).transpose().getArray()[0];
    	}
        double t =  muExp.minus(muBarOne).norm2();
         ///System.err.println(t);
        if(t <  epsilon){
           System.out.println("DONE");
           learning = false;
           weights = oldWeights;
        }
     /*   for(int x =0; x < weights.length; x++){
        	System.out.print(weights[x] + " , ");
        }
        System.out.print("\n");
	*/	
	
	}

	//convex hull check.  See Abbeel's thesis for why this is here.  That singular exception happens a lot.  Don't worry about it too much.
	protected boolean checkConvex(Matrix a, Matrix b, Matrix check){
		int d = dimensions;
		Matrix x = new Matrix(2,d);
		//System.out.println(check.getRowDimension() + "  " + check.getColumnDimension());
		x.setMatrix(0, 0, 0, d-1, a.transpose());
		x.setMatrix(1, 1, 0, d-1, b.transpose());
		Matrix mid;
		try{
			//x.print(new DecimalFormat(), 5);
			mid = x.transpose().times(x).inverse();
		} catch(Exception ex){ //matrix was singular.  At some point I convinced myself that returning true is right here because of geometry.
			//System.out.println("Hit a singular matrix, moving on anyway.");
			return true;
		}
		Matrix v = x.times(mid).times(x.transpose());
		double m = v.get(0, 0);
		for(int r = 0; r < v.getRowDimension(); r++) {
			if(v.get(r,r) > m) 
				m = v.get(r,r);
		}
		return (check.transpose().times(mid).times(check).get(0, 0) <= m);
	}
	
	
	private void runAutonomousTrajectory(OOMDPState startState) {
		Environment environment = Interface.getCurrentEnvironment();
		
		OOMDPState currentState = startState;
		agentBegin(startState);
		for (int i = 0; i < 30; i++) {
			String action = null; // TODO: Plan if no action
			currentState = environment.simulateAction(currentState, action);
			agentStep(currentState);
			// TODO Get the next action, etc
		}
		agentEnd();
	}
	
	public void train() {
		Random r = new Random();
		
		for (int i = 0; i < 70; i++) {
			// Sample a random start state from the teacher traces
			OOMDPState startState = teacherTraces.get(r.nextInt(teacherTraces.size())).get(0);

			// Run a trajectory from that start state
			runAutonomousTrajectory(startState);
		}
	}

	// TODO This wasn't right before, but is it right now?
	public double getReward(OOMDPState s) {
		int[] si = stateToArray(s);
		return dot(weights, toDouble(si)[0]); 
	}
	
	public double getHeuristic(OOMDPState s) {
		return 0.0; // IRL doesn't have one  TODO: But 0 isn't admissible!
	}
	
	@Override
	public Response perform(MDPState startState, int executionLimit) {
		// TODO I'll fill this in, but I assume it will look basically 
		// just like runAutonomousTrajectory but without updating anything
		
		// and of course we will do planning
		
		return null;
	}

	public int[][] traceToArray(List<OOMDPState> trace) {
		int[][] result = new int[trace.size()][trace.get(0).getRelations().size()];
		
		for (int i = 0; i < trace.size(); i++) {
			List<Relation> relations = trace.get(i).getRelations();
			for (int j = 0; j < relations.size(); j++) {
				result[i][j] = (relations.get(i).value ? 1 : 0);
			}
		}
		
		return result;
	}
	
	public int[] stateToArray(OOMDPState s) {
		List<Relation> relations = s.getRelations();
		int[] result = new int[relations.size()];

		for (int i = 0; i < relations.size(); i++) {
			result[i] = (relations.get(i).value ? 1 : 0);
		}
		
		return result;
	}
	
	@Override
	public boolean isReady() {
		return convergenceAchieved;
	}

	
	// TODO: Need to lift some of these to an abstract class that isn't FSM-specific
	@Override
	public String getLexicalForm() {
		return verb_;
	}

	@Override
	public String[] getArgumentArray() {
		return arguments_.toArray(new String[0]);
	}

	@Override
	public ArrayList<String> getArguments() {
		return arguments_;
	}

	@Override
	public String getIdentifierString() {
		return Joiner.on(",").join(Lists.asList(getLexicalForm(), getArgumentArray()));
	}
	
	@Override
	public void addPositiveInstances(List<List<OOMDPState>> traces) {
		throw new RuntimeException("NOT SUPPORTED");
	}

	@Override
	public void addNegativeInstances(List<List<OOMDPState>> traces) {
		throw new RuntimeException("NOT SUPPORTED");
	}

	@Override
	public void addNegativeInstance(List<OOMDPState> trace) {
		throw new RuntimeException("NOT SUPPORTED");
	}
}
