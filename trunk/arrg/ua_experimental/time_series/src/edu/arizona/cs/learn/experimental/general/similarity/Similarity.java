package edu.arizona.cs.learn.experimental.general.similarity;

import java.util.List;

import edu.arizona.cs.learn.experimental.general.Symbol;
import edu.arizona.cs.learn.experimental.general.values.Value;

public enum Similarity {

	tanimoto {
		@Override
		public double similarity(Symbol A, Symbol B) {
			throw new RuntimeException("Not yet implemented!");
		}
	},
	cosine {
		@Override
		public double similarity(Symbol A, Symbol B) {
			if (A.size() != B.size())
				throw new RuntimeException("Key sizes do not match\n" + A.key() + "\n" + B.key());

			double numer = 0;
			double aLength = 0;
			double bLength = 0;
			
			List<Value> aKey = A.key();
			List<Value> bKey = B.key();
			for (int i = 0; i < aKey.size(); ++i) { 
				Value av = aKey.get(i);
				Value bv = bKey.get(i);

				if (!av.considerDistance() || !bv.considerDistance())
					continue;
					
				numer += av.multiply(bv);
				
				aLength += av.multiply(av);
				bLength += bv.multiply(bv);
			}
			
			double cosine = numer / (aLength * bLength);
			
			// The resulting similarity ranges from −1 meaning exactly opposite, 
			// to 1 meaning exactly the same, with 0 usually indicating independence, 
			// and in-between values indicating intermediate similarity or dissimilarity.
			
			// To make the value be between 0 and 1, we add 1 to the similarity and divide by 2
			return (cosine + 1.0) / 2.0;
		}
	};

	/**
	 * The method will return the similarity of the two objects.
	 * A value of 0 is identical and a value of 1 is maximally
	 * dissimilar
	 * @param A
	 * @param B
	 * @return
	 */
	public abstract double similarity(Symbol A, Symbol B); 
}
