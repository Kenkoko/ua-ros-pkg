package edu.arizona.cs.learn.algorithm.alignment.model;

public class Cons {
	public char value;
	public Cons next;
	
	public Cons() { 
		next = null;
	}
	
	public Cons(char value, Cons next) { 
		this.value = value;
		this.next = next;
	}
}
