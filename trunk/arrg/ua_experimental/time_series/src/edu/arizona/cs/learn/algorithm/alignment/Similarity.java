package edu.arizona.cs.learn.algorithm.alignment;

import java.util.List;

import edu.arizona.cs.learn.timeseries.model.symbols.ComplexSymbol;
import edu.arizona.cs.learn.timeseries.model.symbols.StringSymbol;
import edu.arizona.cs.learn.timeseries.model.symbols.Symbol;
import edu.arizona.cs.learn.timeseries.model.values.Value;

public enum Similarity {

	tanimoto {
		@Override
		public double similarity(Symbol A, Symbol B) {
			if (!(A instanceof ComplexSymbol) || !(B instanceof ComplexSymbol))
				throw new RuntimeException("tanimoto coefficient is only defined for ComplexSymbols : " + 
						A.getClass().getSimpleName() + " -- " + B.getClass().getSimpleName());
			
			ComplexSymbol cA = (ComplexSymbol) A;
			ComplexSymbol cB = (ComplexSymbol) B;
			
			if (cA.size() != cB.size())
				throw new RuntimeException("Key sizes do not match\n" + cA.key() + "\n" + cB.key());

			double numer = 0;
			double aLength = 0;
			double bLength = 0;

			List<Value> aKey = cA.key();
			List<Value> bKey = cB.key();
			for (int i = 0; i < aKey.size(); ++i) { 
				Value av = aKey.get(i);
				Value bv = bKey.get(i);

				if (!av.considerDistance() || !bv.considerDistance())
					continue;
					
				numer += av.multiply(bv);
				
				aLength += av.multiply(av);
				bLength += bv.multiply(bv);
			}
			
			return numer / (aLength + bLength - numer);
		}
	},
	cosine {
		@Override
		public double similarity(Symbol A, Symbol B) {
			// cosine similarity is only defined for ComplexSymbols
			if (!(A instanceof ComplexSymbol) || !(B instanceof ComplexSymbol))
				throw new RuntimeException("cosine similarity is only defined for ComplexSymbols : " + 
						A.getClass().getSimpleName() + " -- " + B.getClass().getSimpleName());
			
			ComplexSymbol cA = (ComplexSymbol) A;
			ComplexSymbol cB = (ComplexSymbol) B;
			
			if (cA.size() != cB.size())
				throw new RuntimeException("Key sizes do not match\n" + cA.key() + "\n" + cB.key());

			double numer = 0;
			double aLength = 0;
			double bLength = 0;
			
			List<Value> aKey = cA.key();
			List<Value> bKey = cB.key();
			for (int i = 0; i < aKey.size(); ++i) { 
				Value av = aKey.get(i);
				Value bv = bKey.get(i);

				if (!av.considerDistance() || !bv.considerDistance())
					continue;
					
				numer += av.multiply(bv);
				
				aLength += av.multiply(av);
				bLength += bv.multiply(bv);
			}
			
			double cosine = numer / (Math.sqrt(aLength) * Math.sqrt(bLength));
			
			// The resulting similarity ranges from −1 meaning exactly opposite, 
			// to 1 meaning exactly the same, with 0 usually indicating independence, 
			// and in-between values indicating intermediate similarity or dissimilarity.
			
			// To make the value be between 0 and 1, we add 1 to the similarity and divide by 2
			return (cosine + 1.0) / 2.0;
		}
	},
	strings {
		@Override
		public double similarity(Symbol A, Symbol B) {
			// cosine similarity is only defined for ComplexSymbols
			if (!(A instanceof StringSymbol) || !(B instanceof StringSymbol))
				throw new RuntimeException("strings similarity is only defined for StringSymbols : " + 
						A.getClass().getSimpleName() + " -- " + B.getClass().getSimpleName());
			
			StringSymbol cA = (StringSymbol) A;
			StringSymbol cB = (StringSymbol) B;

			if (cA.getKey().equals(cB.getKey()))
				return 1;
			return 0;
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
