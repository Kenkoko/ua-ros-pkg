package edu.arizona.verbs.shared;

import java.util.Map;

public interface Remappable<T> {
	
	// Remap returns a new object of the same type 
	// with all of the "names" remapped according to nameMap
	T remap(Map<String, String> nameMap);
	
}
