package edu.arizona.verbs.shared;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Map;


public class Relation implements Remappable<Relation>, Comparable<Relation>{
	// Treating this as a data container, so public members
	// However, it is immutable, so should probably enforce that
	public String relation;
	public ArrayList<String> objectNames;
	public boolean value;
	private String cachedString = null;
	
	public Relation() {
		relation = new String();
		objectNames = new ArrayList<String>();
		value = false;
		toString();
	}
	
	public Relation(String relation, ArrayList<String> objectNames, boolean value) {
		this.relation = relation;
		this.objectNames = objectNames;
		this.value = value;
		toString();
	}
	
	@Deprecated
	public Relation(String relation, String[] objectNames, boolean value) {
		this.relation = relation;
		this.objectNames = new ArrayList<String>(Arrays.asList(objectNames));
		this.value = value;
		toString();
	}
	
	public String[] getObjectNameArray() {
		return objectNames.toArray(new String[0]);
	}
	
	public ArrayList<String> getObjectNames() {
		return objectNames;
	}
	
	@Override
	public String toString() {
		if (cachedString == null) { 
			StringBuffer result = new StringBuffer(relation);
			result.append("(");
			for (int i = 0; i < objectNames.size(); i++) {
				result.append(objectNames.get(i));
				if (i < objectNames.size() - 1) {
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
		for (String oldName : objectNames) {
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
