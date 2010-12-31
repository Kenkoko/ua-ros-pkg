package edu.arizona.cs.learn.timeseries.model.symbols;

import java.util.ArrayList;
import java.util.List;

import org.dom4j.Element;

import edu.arizona.cs.learn.timeseries.model.values.Value;

public class ComplexSymbol extends Symbol {
	private List<Value> _key;

	public ComplexSymbol(List<Value> key, double weight) {
		super(weight);

		_key = key;
	}
	
	public ComplexSymbol(List<Value> key) {
		this(key, 1.0);
	}

	public List<Value> key() {
		return _key;
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
	
	public String toString() { 
		return _key.toString();
	}
	
	public boolean equals(Object o) {
		if (!(o instanceof ComplexSymbol)) {
			return false;
		}
		ComplexSymbol other = (ComplexSymbol) o;
		return key().equals(other.key());
	}
	
	@Override
	public Symbol copy() {
		List<Value> key = new ArrayList<Value>();
		for (Value v : _key)
			key.add(v.copy());
		
		return new ComplexSymbol(key, _weight);
	}

	@Override
	public Symbol merge(Symbol B) { 
		if (!(B instanceof ComplexSymbol)) 
			throw new RuntimeException("Can only merge symbols of the same type! ComplexSymbol -- " + B.getClass().getSimpleName());

		ComplexSymbol cB = (ComplexSymbol) B;
		ComplexSymbol newSymbol = (ComplexSymbol) copy();
		newSymbol.increment(B.weight());
		
		for (int i = 0; i < newSymbol._key.size(); ++i) { 
			Value v1 = newSymbol._key.get(i);
			Value v2 = cB._key.get(i);
			
			v1.merge(v2);
		}
		return newSymbol;
	}
	
	@Override
	public String latex() { 
		throw new RuntimeException("Not yet implemented!");
	}
	
	@Override
	public void toXML(Element e) {
		Element sElement = e.addElement("symbol")
			.addAttribute("class", ComplexSymbol.class.getSimpleName())
			.addAttribute("weight", _weight + "");

		for (Value v : _key) { 
			v.toXML(sElement);
		}
	}

	public static ComplexSymbol fromXML(Element e) {
		double weight = Double.parseDouble(e.attributeValue("weight"));
		List<Value> values = new ArrayList<Value>();

		List<?> vList = e.elements("value");
		for (int i = 0; i < vList.size(); ++i) { 
			Element ve = (Element) vList.get(i);
			
			values.add(Value.fromXML(ve));
		}
		return new ComplexSymbol(values, weight);
	}
}