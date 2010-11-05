package edu.arizona.verbs.util;

import java.util.Map;

import edu.arizona.verbs.mdp.Relation;

public class Remapper {
	public static Relation remapRelation(Relation r, Map<String,String> nameMap) {
		String[] remappedNames = new String[r.objectNames.size()];
		for (int i = 0; i < remappedNames.length; i++) {
			String oldName = r.objectNames.get(i);
			if (nameMap.containsKey(oldName)) {
				remappedNames[i] = nameMap.get(r.objectNames.get(i));
			} else {
				remappedNames[i] = oldName;
			}
		}
		return new Relation(r.relation, remappedNames, r.value);
	}
}
