package edu.arizona.verbs.shared;

import java.util.Arrays;
import java.util.List;
import java.util.Map;
import java.util.Vector;


public class Relation implements Remappable<Relation>, Comparable<Relation>{
	// Treating this as a data container, so public members
	// However, it is immutable, so should probably enforce that
	public String relation;
	public List<String> objectNames;
	public boolean value;
	private String cachedString = null;
	
	public Relation() {
		relation = new String();
		objectNames = new Vector<String>();
		value = false;
		toString();
	}
	
	public Relation(String relation, String[] objectNames, boolean value) {
		this.relation = relation;
		this.objectNames = Arrays.asList(objectNames);
		this.value = value;
		toString();
	}
	
	public String[] getObjectNameArray() {
		return objectNames.toArray(new String[0]);
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
		String[] remappedNames = new String[objectNames.size()];
		for (int i = 0; i < remappedNames.length; i++) {
			String oldName = objectNames.get(i);
			if (nameMap.containsKey(oldName)) {
				remappedNames[i] = nameMap.get(objectNames.get(i));
			} else {
				remappedNames[i] = oldName;
			}
		}
		return new Relation(relation, remappedNames, value);
	}

	@Override
	public int compareTo(Relation other) {
		return this.toString().compareTo(other.toString());
	}
}
