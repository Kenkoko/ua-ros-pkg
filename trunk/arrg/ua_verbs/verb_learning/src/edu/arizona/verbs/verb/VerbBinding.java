package edu.arizona.verbs.verb;

import java.util.LinkedHashMap;

import edu.arizona.verbs.verb.vfsm.FSMVerb;

public class VerbBinding {
	public FSMVerb verb;
	public LinkedHashMap<String,String> binding = new LinkedHashMap<String, String>();
}
