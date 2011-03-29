package edu.arizona.verbs.experiments.evaluation;

import java.io.PrintStream;

public class EvaluationResults {
	public double precision;
	public double recall;
	
	public double f() {
		return F1(precision, recall);
	}
	
	public double F1(double precision, double recall) {
		if (precision == 0 && recall == 0) {
			return 0;
		}
		
		return 2 * ((precision * recall) / (precision + recall));
	}
	
   public void printResults() {
	   printResults(System.out);
   }
   
   public void printResults(PrintStream out) {
	   out.println("Precision\tRecall\tF-score");
	   out.println(precision + "\t" + recall + "\t" + f());
   }
}
