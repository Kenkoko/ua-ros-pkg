package edu.arizona.cs.learn.algorithm.alignment.model;


public class Cell {
	public double score;
	public Cons directions;
	
	public Cell(double score) { 
		this.score = score;
		this.directions = null;
	}
}
