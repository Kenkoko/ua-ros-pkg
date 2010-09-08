package edu.arizona.cs.learn.algorithm.alignment;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import edu.arizona.cs.learn.algorithm.alignment.model.WeightedObject;


public class Report {

	public double score;
	
	public int numMatches;
	public int s1Mismatch;
	public int s2Mismatch;
	
	public int s1Size;
	public int s2Size;
	
	public List<WeightedObject> results1;
	public List<WeightedObject> results2;
	
	public Report() { 
		results1 = new ArrayList<WeightedObject>();
		results2 = new ArrayList<WeightedObject>();
	}
	
	public Report(double score) { 
		this();
		
		this.score = score;
	}
	
	public void add(WeightedObject obj1, WeightedObject obj2) { 
		results1.add(obj1);
		results2.add(obj2);
		
		if (obj1 == null)
			++s2Mismatch;
		if (obj2 == null)
			++s1Mismatch;
		if (obj1 != null && obj2 != null) {
			++numMatches;
		}
	}
	
	public void copy(Report sc) { 
		results1 = new ArrayList<WeightedObject>(sc.results1);
		results2 = new ArrayList<WeightedObject>(sc.results2);
		
		s1Mismatch = sc.s1Mismatch;
		s2Mismatch = sc.s2Mismatch;
	}
	
	public void assign(Report sc) { 
		results1 = sc.results1;
		results2 = sc.results2;

		s1Mismatch = sc.s1Mismatch;
		s2Mismatch = sc.s2Mismatch;
	}
	
	public void finish() { 
		Collections.reverse(results1);
		Collections.reverse(results2);
	}
	
	public String toString() { 
		return score + " " + results1.size() + " " + results2.size() + " --- " + numMatches;
	}
}
