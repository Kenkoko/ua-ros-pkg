package edu.arizona.cs.learn.timeseries.evaluation;

import java.util.HashSet;
import java.util.Set;

public class Exclude {

	public static Set<String> handwritingExcludeSAX;
	public static Set<String> handwritingExcludeSDL;
	
	public static Set<String> excludeSAX;
	public static Set<String> excludeSDL;
	
	static { 
		handwritingExcludeSAX = new HashSet<String>();
		excludeSAX = new HashSet<String>();
		for (int i = 1; i < 8; ++i) {
			handwritingExcludeSAX.add(" " + i + ")");
			excludeSAX.add(" " + i + ")");
		}
		
		handwritingExcludeSDL = new HashSet<String>();
		handwritingExcludeSDL.add(" delta0)");
		handwritingExcludeSDL.add(" delta+)");
		handwritingExcludeSDL.add(" delta-)");

		excludeSDL = new HashSet<String>();
		excludeSDL.add(" down)");
		excludeSDL.add(" up)");
		excludeSDL.add(" stable)");
	}
}
