package edu.arizona.cs.learn.timeseries.model.symbols;

import org.dom4j.Element;

import edu.arizona.cs.learn.timeseries.model.values.Binary;
import edu.arizona.cs.learn.timeseries.model.values.Real;
import edu.arizona.cs.learn.timeseries.model.values.Symbolic;

public abstract class Symbol {

	protected double _weight;

	/**
	 * An unspecified weight defaults to 1.0 so that the
	 * alignment scores work out properly.
	 */
	public Symbol() { 
		_weight = 1.0;
	}
	
	public Symbol(double weight) { 
		_weight = weight;
	}
	
	/**
	 * Return the weight of this symbol.  
	 * @return
	 */
	public double weight() { 
		return _weight;
	}

	public void increment(double value) {
		_weight += value;
	}
	
	/**
	 * Create a copy of this symbol.
	 * @param s
	 * @return
	 */
	public abstract Symbol copy();
	
	/**
	 * Add an XML representation of this symbol
	 * to the given XML element.
	 * @param e
	 */
	public abstract void toXML(Element e);
	
	/**
	 * Merge the two symbols into a single 
	 * symbol. Non-destructive so it returns a 
	 * new symbol.
	 * @param s
	 */
	public abstract Symbol merge(Symbol s);
	
	/**
	 * Generate a latex friendly version of this Symbol
	 * @return
	 */
	public abstract String latex();
	
	/**
	 * Construct a Symbol from the given XML element.
	 * @param e
	 * @return
	 */
	public static Symbol fromXML(Element e) { 
		String className = e.attributeValue("class");
		
		if (Event.class.getSimpleName().equals(className)) {
			return Event.fromXML(e);
		} else if (AllenRelation.class.getSimpleName().equals(className)) { 
			return AllenRelation.fromXML(e);
		} else if (CBA.class.getSimpleName().equals(className)) { 
			return CBA.fromXML(e);
		} else if (ComplexSymbol.class.getSimpleName().equals(className)) { 
			return ComplexSymbol.fromXML(e);
		}

		throw new RuntimeException("Error loading XML Symbol! Unknown class " + className);
	}
}
