package edu.arizona.verbs.mdp;

import java.util.Arrays;
import java.util.List;
import java.util.Vector;

public class Relation {
	// Treating this as a data container, so public members
	public String relation;
	public List<String> objectNames;
	public boolean value;
	
	public Relation() {
		relation = new String();
		objectNames = new Vector<String>();
		value = false;
	}
	
	public Relation(String relation, String[] objectNames, boolean value) {
		this.relation = relation;
		this.objectNames = Arrays.asList(objectNames);
		this.value = value;
	}
	
	public String[] getObjectNameArray() {
		return objectNames.toArray(new String[0]);
	}
	
	@Override
	public String toString() {
		StringBuffer result = new StringBuffer(relation);
		result.append("(");
		for (int i = 0; i < objectNames.size(); i++) {
			result.append(objectNames.get(i));
			if (i < objectNames.size() - 1) {
				result.append(",");
			}
		}
		result.append(")");
		return result.toString();
	}
}
