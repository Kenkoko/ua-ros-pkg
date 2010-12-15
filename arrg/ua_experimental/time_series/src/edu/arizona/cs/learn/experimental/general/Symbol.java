package edu.arizona.cs.learn.experimental.general;

import java.util.ArrayList;
import java.util.List;
import java.util.Set;
import java.util.TreeSet;

import org.dom4j.Element;

import edu.arizona.cs.learn.experimental.general.values.Value;

public class Symbol {
	private List<Value> _key;
	private double _weight;

	public Symbol(List<Value> key, double weight) {
		_key = key;
		_weight = weight;
	}
	
	public Symbol(List<Value> key) {
		this(key, 1.0);
	}

	public List<Value> key() {
		return _key;
	}
	
	public double weight() {
		return _weight;
	}
	
	public double size() { 
		return _key.size();
	}

	/**
	 * Add the example to the set of examples for
	 * this Symbol.  Also update the key for this symbol
	 * to the correct value.
	 * @param example
	 */
	public void addExample(List<Value> example) { 
		for (int i = 0; i < _key.size(); ++i) {
			_key.get(i).merge(example.get(i));
		}
	}
	
	public void increment(double value) {
		_weight += value;
	}
	
	public String toString() { 
		return _key.toString();
	}
	
	public Symbol copy() {
		List<Value> key = new ArrayList<Value>();
		for (Value v : _key)
			key.add(v.copy());
		
		return new Symbol(key, _weight);
	}

	public boolean equals(Object o) {
		if (!(o instanceof Symbol)) {
			return false;
		}
		Symbol other = (Symbol) o;
		return key().equals(other.key());
	}
	
	public void toXML(Element e) {
		Element sElement = e.addElement("symbol");
		sElement.addAttribute("weight", _weight+"");
		for (Value v : _key) { 
			v.toXML(sElement);
		}
	}

	public static Symbol fromXML(Element e) {
		
		double weight = Double.parseDouble(e.attributeValue("weight"));
		List<Value> values = new ArrayList<Value>();

		List<?> vList = e.elements("value");
		for (int i = 0; i < vList.size(); ++i) { 
			Element ve = (Element) vList.get(i);
			
			values.add(Value.fromXML(ve));
		}
		return new Symbol(values, weight);
	}
	
	/**
	 * Compute the dot product of two lists of Doubles
	 * @param A
	 * @param B
	 * @return
	 */
	public static double dot(List<Value> A, List<Value> B) { 
		double d = 0;
		for (int i = 0; i < A.size(); ++i) { 
			d += A.get(i).multiply(B.get(i));
		}
		return d;
	}
	
	/**
	 * Compute the length of the vector.
	 * @param A
	 * @return
	 */
	public static double length(List<Value> A) { 
		return Math.sqrt(lengthSquared(A));
	}
	
	/**
	 * Compute the length squared of the vector
	 * @param A
	 * @return
	 */
	public static double lengthSquared(List<Value> A) { 
		double d = 0.0;
		for (int i = 0; i < A.size(); ++i) 
			d += A.get(i).multiply(A.get(i));
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
			if (!A._key.get(i).considerDistance() || !B._key.get(i).considerDistance())
				exclude.add(i);
		}

		List<Value> subA = new ArrayList<Value>();
		List<Value> subB = new ArrayList<Value>();
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
		
		for (int i = 0; i < newSymbol._key.size(); ++i) { 
			Value v1 = newSymbol._key.get(i);
			Value v2 = B._key.get(i);
			
			v1.merge(v2);
		}
		return newSymbol;
	}
}