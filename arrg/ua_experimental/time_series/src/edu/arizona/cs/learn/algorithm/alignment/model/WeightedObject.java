package edu.arizona.cs.learn.algorithm.alignment.model;

import org.dom4j.Element;

public class WeightedObject {
	private Symbol _key;
	private double _weight;

	public WeightedObject(Symbol key, double weight) {
		_key = key;
		_weight = weight;
	}

	public WeightedObject(Symbol key) {
		this(key, 1.0);
	}

	public Symbol key() {
		return _key;
	}

	public double weight() {
		return _weight;
	}

	public void increment(double value) {
		_weight += value;
	}

	public WeightedObject copy() {
		return new WeightedObject(this._key, _weight);
	}

	public boolean equals(Object o) {
		if (!(o instanceof WeightedObject)) {
			return false;
		}
		WeightedObject other = (WeightedObject) o;
		return key().equals(other.key());
	}

	public String toString() {
		return this._key.toString();
	}

	public void toXML(Element e) {
		Element obj = e.addElement("WeightedObject").addAttribute("weight",_weight+"");
		this._key.toXML(obj);
	}

	public static WeightedObject fromXML(Element e) {
		double weight = Double.parseDouble(e.attributeValue("weight"));
		Symbol s = Symbol.fromXML(e.element("Symbol"));
		return new WeightedObject(s, weight);
	}
}