package edu.arizona.cs.learn;

import java.util.List;

import com.martiansoftware.jsap.FlaggedOption;
import com.martiansoftware.jsap.JSAP;
import com.martiansoftware.jsap.JSAPResult;
import com.martiansoftware.jsap.Switch;

import edu.arizona.cs.learn.timeseries.Experiments;
import edu.arizona.cs.learn.timeseries.evaluation.cluster.Clustering;
import edu.arizona.cs.learn.timeseries.model.SequenceType;
import edu.arizona.cs.learn.util.Ablation;
import edu.arizona.cs.learn.util.Utils;

public class Main {
	public static void main(String[] args) throws Exception {
		JSAP jsap = new JSAP();

		Switch helpSwitch = new Switch("help")
				.setShortFlag('h')
				.setLongFlag("help");

		FlaggedOption taskOption = new FlaggedOption("task")
				.setDefault("comparison")
				.setRequired(true)
				.setLongFlag("task");

		FlaggedOption dataOption = new FlaggedOption("data")
				.setDefault("all")
				.setRequired(true)
				.setLongFlag("data");

		FlaggedOption classifyOption = new FlaggedOption("classify")
				.setDefault("knn")
				.setRequired(false)
				.setLongFlag("classify");

		FlaggedOption sequenceOption = new FlaggedOption("sequence")
				.setDefault("starts")
				.setRequired(false)
				.setLongFlag("sequence");

		Switch pruneSwitch = new Switch("prune")
				.setShortFlag('p')
				.setLongFlag("prune");

		Switch shuffleSwitch = new Switch("shuffle")
				.setShortFlag('s')
				.setLongFlag("shuffle");

		Switch fromFileSwitch = new Switch("fromFile")
				.setShortFlag('f')
				.setLongFlag("fromFile");

		Switch liteSwitch = new Switch("lite")
				.setLongFlag("lite");
		
		FlaggedOption pctOption = new FlaggedOption("min")
				.setStringParser(JSAP.INTEGER_PARSER).setDefault("50")
				.setRequired(false)
				.setLongFlag("prune-pct");

		FlaggedOption kOption = new FlaggedOption("k")
				.setStringParser(JSAP.INTEGER_PARSER).setDefault("10")
				.setRequired(false)
				.setShortFlag('k');

		FlaggedOption threadsOption = new FlaggedOption("threads")
				.setStringParser(JSAP.INTEGER_PARSER)
				.setRequired(false)
				.setDefault(Utils.numThreads+"")
				.setLongFlag("threads");

		FlaggedOption tmpOption = new FlaggedOption("tmpdir")
				.setDefault("/tmp/")
				.setRequired(false)
				.setShortFlag('m');

		FlaggedOption foldsOption = new FlaggedOption("folds")
				.setStringParser(JSAP.INTEGER_PARSER).setDefault("10")
				.setRequired(false)
				.setLongFlag("folds");

		FlaggedOption ablateOption = new FlaggedOption("ablate")
				.setRequired(false)
				.setDefault("none")
				.setShortFlag('a')
				.setLongFlag("ablate");

		FlaggedOption testAblateOption = new FlaggedOption("testAblate")
				.setRequired(false)
				.setDefault("none")
				.setShortFlag('t')
				.setLongFlag("testAblate");
		

		jsap.registerParameter(helpSwitch);
		jsap.registerParameter(taskOption);
		jsap.registerParameter(dataOption);
		jsap.registerParameter(classifyOption);
		jsap.registerParameter(sequenceOption);
		jsap.registerParameter(pctOption);
		jsap.registerParameter(kOption);
		jsap.registerParameter(threadsOption);
		jsap.registerParameter(tmpOption);
		jsap.registerParameter(foldsOption);
		jsap.registerParameter(ablateOption);
		jsap.registerParameter(testAblateOption);
		jsap.registerParameter(pruneSwitch);
		jsap.registerParameter(shuffleSwitch);
		jsap.registerParameter(fromFileSwitch);
		jsap.registerParameter(liteSwitch);

		JSAPResult config = jsap.parse(args);

		if (config.getBoolean("help")) {
			System.out.print("java -jar time-series.jar ");
			System.out.println(jsap.getUsage());

			System.out
					.println("  --task : distance/signatures/prepare/comparison/power");
			System.out.println("  -c: knn cave all");
			return;
		}

		Utils.numThreads = config.getInt("threads");
		Utils.tmpDir = config.getString("tmpdir");
		Utils.excludeSet = Ablation.valueOf(config.getString("ablate")).getExcludeSet();
		Utils.testExcludeSet = Ablation.valueOf(config.getString("testAblate")).getExcludeSet();

		String task = config.getString("task");
		if (task.equals("distance")) {
			List<String> prefix = Utils.getPrefixes(config.getString("data"));
			List<SequenceType> types = SequenceType.get(config.getString("sequence"));
			Clustering.init(prefix, types);
		} else if (task.equals("signatures")) {
			boolean prune = config.getBoolean("prune");

			List<String> prefixes = Utils.getPrefixes(config.getString("data"));
			List<SequenceType> types = SequenceType.get(config.getString("sequence"));

			int folds = config.getInt("folds");
			for (String prefix : prefixes)
				for (SequenceType type : types)
					Experiments.signatures(prefix, type, folds, prune);
		} else {
			if (task.equals("timing")) {
				boolean prune = config.getBoolean("prune");

				List<String> prefixes = Utils.getPrefixes(config.getString("data"));
				List<SequenceType> types = SequenceType.get(config.getString("sequence"));

				int folds = config.getInt("folds");
				for (String prefix : prefixes) {
					for (SequenceType type : types)
						Experiments.signatureTiming(prefix, type, folds, prune);
				}
			} else if (task.equals("prepare")) {
				List<String> prefixes = Utils.getPrefixes(config.getString("data"));
				int folds = config.getInt("folds");

				for (String prefix : prefixes)
					Experiments.selectCrossValidation(prefix, folds);
			} else if (task.equals("comparison")){
				System.out.println("Doing the comparison");
				Experiments.init(config.getString("data"),
						config.getString("classify"),
						config.getString("sequence"), config.getInt("min"),
						config.getInt("k"), config.getInt("folds"),
						config.getBoolean("shuffle"),
						config.getBoolean("fromFile"), 
						config.getBoolean("lite"));
			}
		}
	}
}