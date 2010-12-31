package edu.arizona.cs.learn.timeseries.model.values;

import org.dom4j.Element;

public abstract class Value {

	protected String _variableName;
	
	public Value(String variableName) { 
		_variableName = variableName;
	}
	
	public abstract void merge(Value v);
	
	public abstract boolean considerDistance();

	public abstract double value();
	public abstract double multiply(Value v);
	public abstract double distance(Value v);

	public abstract boolean unknown();
	
	public abstract Value copy();
	
	public abstract void toXML(Element e);
	
	public static Value fromXML(Element e) { 
		String className = e.attributeValue("class");
		
		if (Binary.class.getSimpleName().equals(className)) 
			return Binary.fromXML(e);
		if (Real.class.getSimpleName().equals(className))
			return Real.fromXML(e);
		if (Symbolic.class.getSimpleName().equals(className))
			return Symbolic.fromXML(e);
		
		throw new RuntimeException("Error loading XML! Unknown class " + className);
	}
}
