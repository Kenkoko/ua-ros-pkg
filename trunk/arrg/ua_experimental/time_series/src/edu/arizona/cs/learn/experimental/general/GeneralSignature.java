package edu.arizona.cs.learn.experimental.general;

import java.io.FileWriter;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

import org.apache.log4j.Logger;
import org.dom4j.Document;
import org.dom4j.DocumentHelper;
import org.dom4j.Element;
import org.dom4j.io.OutputFormat;
import org.dom4j.io.SAXReader;
import org.dom4j.io.XMLWriter;

import edu.arizona.cs.learn.algorithm.alignment.GeneralAlignment;
import edu.arizona.cs.learn.algorithm.alignment.Params;
import edu.arizona.cs.learn.algorithm.alignment.Report;
import edu.arizona.cs.learn.algorithm.alignment.Similarity;
import edu.arizona.cs.learn.timeseries.model.symbols.ComplexSymbol;
import edu.arizona.cs.learn.timeseries.model.symbols.Symbol;

public class GeneralSignature {
	private static Logger logger = Logger.getLogger(GeneralSignature.class);

	private String _key;
	private List<Symbol> _signature;

	private int _count;
	private Params _params;

	public GeneralSignature(String key, Similarity similarity) {
		_key = key;
		_count = 0;
		_signature = new ArrayList<Symbol>();

		_params = new Params();
		_params.setMin(0, 0);
		_params.setBonus(1.0D, 0.0D);
		_params.setPenalty(-1.0D, 0.0D);
		_params.similarity = similarity;
	}

	public String key() {
		return _key;
	}

	public List<Symbol> signature() {
		return _signature;
	}

	public int trainingSize() {
		return _count;
	}

	public void update(List<Symbol> seq) {
		_count += 1;

		_params.seq1 = _signature;
		_params.seq2 = seq;
		
		Report report = GeneralAlignment.alignCheckp(_params);
		_signature = GeneralAlignment.combineAlignments(report.results1, report.results2);
	}
	
	public void train(List<List<Symbol>> sequences) {
		for (int i = 0; i < sequences.size(); i++) {
			update(sequences.get(i));
		}
	}

	/**
	 * Combine this signature with another signature to create
	 * a super signature.
	 * @param s2
	 */
	public void merge(GeneralSignature s2) {
		throw new RuntimeException("Not yet implemented!!");
	}

	/**
	 * Prune the items in this signature and return a new signature
	 * that is composed of the frequently occurring items.
	 * @param min
	 * @return
	 */
	public GeneralSignature prune(int min) {
		GeneralSignature s = new GeneralSignature(_key, _params.similarity);
		s._count = _count;

		Set<Integer> ignore = new HashSet<Integer>();
		List<Symbol> sequence = new ArrayList<Symbol>();
		for (int i = 0; i < _signature.size(); i++) {
			Symbol obj = _signature.get(i);
			if (obj.weight() > min)
				sequence.add(obj.copy());
			else {
				ignore.add(Integer.valueOf(i));
			}
		}

		s._signature = sequence;
		return s;
	}
	
	/**
	 * Write this signature to an XML file so that we don't constantly
	 * have to relearn this signature.
	 * @param file
	 */
	public void toXML(String file) {
		Document document = DocumentHelper.createDocument();
		Element root = document.addElement("Signature ")
				.addAttribute("key", _key)
				.addAttribute("count", _count+"")
				.addAttribute("similarity", _params.similarity.toString());

		for (Symbol obj : this._signature) {
			obj.toXML(root);
		}

		try {
			OutputFormat format = OutputFormat.createPrettyPrint();
			XMLWriter writer = new XMLWriter(new FileWriter(file), format);
			writer.write(document);
			writer.close();
		} catch (Exception e) {
			e.printStackTrace();
		}
	}	

	/**
	 * Reconstruct a signature from an XML file
	 * @param file
	 * @return
	 */
	public static GeneralSignature fromXML(String file) {
		Document document = null;
		try {
			SAXReader reader = new SAXReader();
			document = reader.read(file);
		} catch (Exception e) {
			e.printStackTrace();
		}
		Element root = document.getRootElement();

		String key = root.attributeValue("key");
		Similarity similarity = Similarity.valueOf(root.attributeValue("similarity"));
		GeneralSignature s = new GeneralSignature(key, similarity);
		s._count = Integer.parseInt(root.attributeValue("count"));

		List list = root.elements("symbol");
		for (Object o : list) { 
			Element woe = (Element) o;
			s._signature.add(ComplexSymbol.fromXML(woe));
		}
		return s;
	}
}