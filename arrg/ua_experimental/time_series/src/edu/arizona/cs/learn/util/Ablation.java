package edu.arizona.cs.learn.util;

import java.util.HashSet;
import java.util.Set;

public enum Ablation {
	none {
		public Set<String> getExcludeSet() { 
			return new HashSet<String>();
		}
	},
	sdl {
		public Set<String> getExcludeSet() { 
			return excludeSDL;
		}
	},
	sax {
		public Set<String> getExcludeSet() { 
			return excludeSAX;
		}
	},
	ww2d { 
		public Set<String> getExcludeSet() { 
			// exclude everything internal ....
			return new HashSet<String>();
		}
	},
	ww3d { 
		public Set<String> getExcludeSet() { 
			// exclude motor commands ....
			return new HashSet<String>();
		}
	};

	public static Set<String> excludeSAX;
	public static Set<String> excludeSDL;

	static
	{
		excludeSAX = new HashSet<String>();
		for (int i = 1; i < 8; i++) {
			excludeSAX.add(" " + i + ")");
		}

		excludeSDL = new HashSet<String>();
		excludeSDL.add(" down)");
		excludeSDL.add(" up)");
		excludeSDL.add(" stable)");
	}

	public abstract Set<String> getExcludeSet();
}