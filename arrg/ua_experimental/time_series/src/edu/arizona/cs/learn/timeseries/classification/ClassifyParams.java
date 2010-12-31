package edu.arizona.cs.learn.timeseries.classification;

import edu.arizona.cs.learn.algorithm.alignment.Similarity;
import edu.arizona.cs.learn.timeseries.model.SequenceType;

/**
 * This class contains all of the possible params
 * that may be set when initializing a classifier.
 * @author wkerr
 *
 */
public class ClassifyParams {
	
	/** For k-nearest-neigbhors classification */
	public int k = 10;
	public boolean weighted = true;
	

	// These below are for the CAVE classifier.  
	// prunePct is the minimum percent that a symbol must been seen within
	// the signature to survive pruning.  From files determines whether
	// fromFiles decides whether to load the signatures from file
	// or do the learning on the fly.
	// type and folds are only necessary when loading from file so that we can
	// pull up the right signature.
	public double prunePct = 0.5;
	public boolean fromFiles = false;
	public boolean incPrune = true;

	public SequenceType type = SequenceType.allen;
	public int folds;
	
	public String method = null;
	
	public Similarity similarity = Similarity.strings;
}
