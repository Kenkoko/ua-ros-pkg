package edu.arizona.cs.learn.util;

public class Range {
	public int min;
	public int max;  // not inclusive
	
	public Range(int min, int max) { 
		this.min = min;
		this.max = max;
	}
	
	public String toString() { 
		return "[" + min + "," + max + "]";
	}
}
