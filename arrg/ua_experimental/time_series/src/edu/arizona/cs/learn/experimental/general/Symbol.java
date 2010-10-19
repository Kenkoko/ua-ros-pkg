package edu.arizona.cs.learn.experimental.general;

import java.util.ArrayList;
import java.util.List;
import java.util.Set;
import java.util.TreeSet;

import org.dom4j.Element;

public class Symbol {
	private List<Double> _key;
	private double _weight;

	private List<List<Double>> _examples;
	
	private Symbol(List<Double> key, double weight, List<List<Double>> examples) {
		_key = key;
		_weight = weight;
		
		_examples = examples;
	}
	
	public Symbol(List<Double> key, double weight) {
		this(key, weight, new ArrayList<List<Double>>());
		_examples.add(key);
	}

	public Symbol(List<Double> key) {
		this(key, 1.0);
	}

	public List<Double> key() {
		return _key;
	}
	
	public double weight() {
		return _weight;
	}

	/**
	 * Add the example to the set of examples for
	 * this Symbol.  Also update the key for this symbol
	 * to the correct value.
	 * @param example
	 */
	public void addExample(List<Double> example) { 
		_examples.add(new ArrayList<Double>(example));
		
		for (int i = 0; i < _key.size(); ++i) { 
			if (_key.get(i) == Double.NaN) 
				continue;
			
			if (Double.compare(_key.get(i), example.get(i)) != 0) 
				_key.set(i, Double.NaN);
		}
	}
	
	public void increment(double value) {
		_weight += value;
	}
	
	public String toString() { 
		return _key.toString();
	}
	
	public Symbol copy() {
		return new Symbol(new ArrayList<Double>(_key), _weight, _examples);
	}

	public boolean equals(Object o) {
		if (!(o instanceof Symbol)) {
			return false;
		}
		Symbol other = (Symbol) o;
		return key().equals(other.key());
	}
	
	public void toXML(Element e) {
		throw new RuntimeException("Not supported yet");
	}

	public static Symbol fromXML(Element e) {
		throw new RuntimeException("Not supported yet");
	}
	
	/**
	 * Compute the dot product of two lists of Doubles
	 * @param A
	 * @param B
	 * @return
	 */
	public static double dot(List<Double> A, List<Double> B) { 
		double d = 0;
		for (int i = 0; i < A.size(); ++i) { 
			d += (A.get(i) * B.get(i));
		}
		return d;
	}
	
	/**
	 * Compute the length of the vector.
	 * @param A
	 * @return
	 */
	public static double length(List<Double> A) { 
		return Math.sqrt(lengthSquared(A));
	}
	
	/**
	 * Compute the length squared of the vector
	 * @param A
	 * @return
	 */
	public static double lengthSquared(List<Double> A) { 
		double d = 0.0;
		for (int i = 0; i < A.size(); ++i) 
			d += (A.get(i) * A.get(i));
		return d;
	}



	/**
	 * Although A and B consist of Vectors of doubles the 
	 * Tanimoto Coefficient considers them to be Binary
	 * @param A
	 * @param B
	 * @return
	 */
	public static double tanimotoCoefficient(Symbol A, Symbol B) { 
		if (A._key.size() != B._key.size())
			throw new RuntimeException("Key sizes do not match\n" + A._key + "\n" + B._key);
		
		// we have to subsequence the keys since they may have
		// NaN within them.
		Set<Integer> exclude = new TreeSet<Integer>();
		for (int i = 0; i < A._key.size(); ++i) {
			if (Double.compare(Double.NaN, A._key.get(i)) == 0)
				exclude.add(i);
			
			if (Double.compare(Double.NaN, B._key.get(i)) == 0)
				exclude.add(i);
		}
		
		List<Double> subA = new ArrayList<Double>();
		List<Double> subB = new ArrayList<Double>();
		for (int i = 0; i < A._key.size(); ++i) { 
			if (exclude.contains(i))
				continue;

			subA.add(A._key.get(i));
			subB.add(B._key.get(i));
		}
		
		double dot = dot(subA, subB);
		double lengthA = lengthSquared(subA);
		double lengthB = lengthSquared(subB);

		return dot / (lengthA + lengthB - dot);
	}

	/**
	 * Merge two Symbols into a single new Symbol.
	 * @param A
	 * @param B
	 * @return
	 */
	public static Symbol merge(Symbol A, Symbol B) { 
//		System.out.println("---- Merge: \n   " + A._key + "\n   " + B._key + " " + B._examples.size());
		Symbol newSymbol = A.copy();
		newSymbol.increment(B._weight);
		for (List<Double> example : B._examples) {
			newSymbol.addExample(example);
		}
		return newSymbol;
	}
}