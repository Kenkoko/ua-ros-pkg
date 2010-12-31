package edu.arizona.cs.learn.timeseries.model.symbols;

import java.util.List;

import org.dom4j.Element;

import edu.arizona.cs.learn.timeseries.model.Interval;

public class StringSymbol extends Symbol {
	private String _name;

	public StringSymbol() {
		this("", 1.0);
	}
	
	public StringSymbol(double weight) { 
		super(weight);
	}

	public StringSymbol(String name) {
		this(name, 1.0);
	}
	
	public StringSymbol(String name, double weight) { 
		super(weight);
		
		_name = name;
	}

	public String getKey() {
		return _name;
	}
	
	public boolean equals(Object o) {
		if (!(o instanceof StringSymbol)) {
			return false;
		}
		StringSymbol symbol = (StringSymbol) o;
		return _name.equals(symbol._name);
	}
	
	public String toString() { 
		return _name;
	}
	
	public String latex() { 
		throw new RuntimeException("Latex is not defined for base class Symbol");
	}

	public List<String> getProps() {
		throw new RuntimeException("No propositions associated with a base class Symbol");
	}

	public List<Interval> getIntervals() {
		throw new RuntimeException("No intervals associated with a base class symbol");
	}

	public void toXML(Element e) {
		throw new RuntimeException("This constructor should never be used for XML purposes");
	}

	@Override
	public Symbol copy() {
		return new StringSymbol(_name, _weight);
	}

	@Override
	public Symbol merge(Symbol B) {
		if (!(B instanceof StringSymbol)) 
			throw new RuntimeException("Can only merge symbols of the same type! StringSymbol -- " + B.getClass().getSimpleName());

		StringSymbol sB = (StringSymbol) B;
		Symbol newSymbol = copy();
		newSymbol.increment(sB.weight());
		
		return newSymbol;
	}
}