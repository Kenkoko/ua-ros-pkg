package edu.arizona.cs.learn.timeseries.model.values;

import java.util.ArrayList;
import java.util.List;

import org.dom4j.Element;

import edu.arizona.cs.learn.util.XMLUtils;

public class Symbolic extends Value {

	private String _uniqueValue;
	private List<String> _values;
	
	private List<String> _symbols;
	private double[][] _costMatrix;
	
	/**
	 * Construct a new Symbolic value where the string
	 * is the same as the symbol and the string "unknown" 
	 * means that the value is unknown
	 * @param value
	 */
	public Symbolic(String variableName, String value) { 
		super(variableName);
		_uniqueValue = value;
		_values = new ArrayList<String>();
		_values.add(value);
	}
	
	/**
	 * Construct a new Symbolic value where the string 
	 * is the same as the symbol and the string "unknown"
	 * means that the value is unknown.
	 * @param value
	 * @param symbols
	 * 	  The array of all possible symbols.  Used to find the
	 *    indexes into the cost matrix.
	 * @param costMatrix
	 *    The cost matrix defines the distance between any 
	 *    two possible symbols.
	 */
	public Symbolic(String variableName, String value, List<String> symbols, double[][] costMatrix) { 
		this(variableName, value);
		
		_symbols = new ArrayList<String>(symbols);
		_costMatrix = costMatrix;
	}
	
	/**
	 * Called by the copy method to create a copy of this value.
	 * @param uniqueValue
	 * @param values
	 * @param symbols
	 * @param costMatrix
	 */
	private Symbolic(String variableName, String uniqueValue, List<String> values, List<String> symbols, double[][] costMatrix) { 
		super(variableName);
		_uniqueValue = uniqueValue;
		_values = new ArrayList<String>(values);
		
		_symbols = symbols;
		_costMatrix = costMatrix;
	}
	
	@Override
	public void merge(Value v) {
		if (!(v instanceof Symbolic))
			throw new RuntimeException("Cannot merge different types: Symbolic - " + 
					v.getClass().getSimpleName());
		
		Symbolic s = (Symbolic) v;
		// First add in all of the new values.
		_values.addAll(s._values);
		
		// Now check to see if we are still unique.
		if (_uniqueValue != s._uniqueValue) { 
			_uniqueValue = "unknown";
		}

	}
	
	@Override
	public boolean considerDistance() {
		return !unknown();
	}
	
	@Override
	public double distance(Value v) {
		if (!(v instanceof Symbolic))
			throw new RuntimeException("Distance undefined between: Symbolic - " + 
					v.getClass().getSimpleName());

		// we are only considering distance when size
		// is equal to 1.... otherwise I throw an exception
		if (!unknown() || !v.unknown())
			throw new RuntimeException("Distance undefined between these two symbolic values");
		
		Symbolic s = (Symbolic) v;
		if (_costMatrix == null) {
			if (s._uniqueValue.equals(_uniqueValue))
				return 0;
			else
				return 1;
		} 
		
		int idx1 = _symbols.indexOf(_uniqueValue);
		int idx2 = _symbols.indexOf(s._uniqueValue);
			
		return _costMatrix[idx1][idx2];
	}

	@Override
	public boolean unknown() {
		return "unknown".equals(_uniqueValue);
	}

	@Override
	public Value copy() {
		return new Symbolic(_variableName, _uniqueValue, _values, _symbols, _costMatrix);
	}
	
	@Override
	public String toString() { 
		return "[" + _values.size() + "] - " + _uniqueValue;
	}

	@Override
	public double value() {
		return Double.NaN;
	}

	@Override
	public double multiply(Value v) {
		return Double.NaN;
	}
	
	@Override
	public void toXML(Element e) { 			
		Element sElement = e.addElement("value")
			.addAttribute("class", Symbolic.class.getSimpleName())
			.addAttribute("variable", _variableName)
			.addAttribute("value", _uniqueValue+"")
			.addAttribute("values", XMLUtils.toString(_values));

		if (_costMatrix != null) { 
			sElement.addElement("symbols", XMLUtils.toString(_symbols));
			XMLUtils.toXML(sElement, _costMatrix);
		} 
	}
	
	public static Value fromXML(Element e) { 
		String varName = e.attributeValue("variable");
		String value = e.attributeValue("value");
		
		String csList = e.attributeValue("values");
		String[] tokens = csList.split("[,]");
		List<String> values = new ArrayList<String>();
		for (String tok : tokens) 
			values.add(tok);
		
		if (e.element("symbols") != null) { 
			throw new RuntimeException("Not yet implemented!");
		}
		return new Symbolic(varName, value, values, null, null);
	}	
	
}
