package edu.arizona.verbs.shared;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Map;


public class Relation implements Remappable<Relation>, Comparable<Relation>{
	// Treating this as a data container, so public members
	// However, it is immutable, so should probably enforce that // TODO Not exactly true
	private String relation;
	private ArrayList<String> arguments;
	private boolean value;
	private String cachedString = null;
	
	public Relation() {
		relation = new String();
		arguments = new ArrayList<String>();
		value = false;
		toString();
	}
	
	public Relation(String relation, ArrayList<String> objectNames, boolean value) {
		this.relation = relation;
		this.arguments = objectNames;
		this.value = value;
		toString();
	}
	
	@Deprecated
	public Relation(String relation, String[] objectNames, boolean value) {
		this.relation = relation;
		this.arguments = new ArrayList<String>(Arrays.asList(objectNames));
		this.value = value;
		toString();
	}
	
	public String[] getObjectNameArray() {
		return arguments.toArray(new String[0]);
	}
	
	public ArrayList<String> getObjectNames() {
		return arguments;
	}
	
	public String getRelation() {
		return relation;
	}

	public void setRelation(String relation) {
		this.relation = relation;
		this.cachedString = null;
	}

	public ArrayList<String> getArguments() {
		return arguments;
	}

	public void setArguments(ArrayList<String> arguments) {
		this.arguments = arguments;
		this.cachedString = null;
	}

	public boolean getValue() {
		return value;
	}

	public void setValue(boolean value) {
		this.value = value;
		this.cachedString = null;
	}

	@Override
	public String toString() {
		if (cachedString == null) { 
			StringBuffer result = new StringBuffer(relation);
			result.append("(");
			for (int i = 0; i < arguments.size(); i++) {
				result.append(arguments.get(i));
				if (i < arguments.size() - 1) {
					result.append(",");
				}
			}
			result.append(")");
			cachedString = result.toString();
		} 
			
		return cachedString;
	}

	@Override
	public Relation remap(Map<String, String> nameMap) {
		ArrayList<String> remappedNames = new ArrayList<String>();
		for (String oldName : arguments) {
			if (nameMap.containsKey(oldName)) {
				remappedNames.add(nameMap.get(oldName));
			} else {
				remappedNames.add(oldName);
			}
		}
		return new Relation(relation, remappedNames, value);
	}

	@Override
	public int compareTo(Relation other) {
		return this.toString().compareTo(other.toString());
	}
}
