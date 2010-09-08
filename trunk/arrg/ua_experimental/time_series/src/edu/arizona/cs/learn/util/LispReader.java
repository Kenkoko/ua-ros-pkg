package edu.arizona.cs.learn.util;

import java.io.FileReader;
import java.io.PushbackReader;
import java.util.ArrayList;
import java.util.List;

import org.apache.log4j.Logger;

public class LispReader {
    private static Logger logger = Logger.getLogger(LispReader.class);

    public static String readSymbol(PushbackReader in) throws Exception { 
		StringBuilder buf = new StringBuilder();
		char c = (char) in.read();
		while (!Character.isWhitespace(c) && c != ')') {
			buf.append(c);
			c = (char) in.read();
		}		
		in.unread(c);
//		logger.debug("ReadSymbol: " + buf.toString());
		return buf.toString().toLowerCase();
    }
    
	/**
	 * read from the input until you read the closing
	 * quotation mark.  
	 * @param in
	 * @return
	 * @throws Exception
	 */
	public static String readString(PushbackReader in) throws Exception { 
		StringBuilder buf = new StringBuilder();
		char c = (char) in.read();
		while (c != '"') {
			buf.append(c);
			c = (char) in.read();
		}		
//		logger.debug("ReadString: " + buf.toString());
		return buf.toString();
	}
	
	public static Integer readNumber(PushbackReader in) throws Exception { 
		StringBuilder buf = new StringBuilder();
		char c = (char) in.read();
		while (!Character.isWhitespace(c) && c != ')') { 
			buf.append(c);
			c = (char) in.read();
		}
		
		in.unread(c);
//		logger.debug("ReadNumber: " + buf.toString());
		return Integer.parseInt(buf.toString());
	}

	/**
	 * Read the input from the given file.
	 * @param fileName
	 * @return
	 * @throws Exception
	 */
	public List<Object> read(String fileName) throws Exception {
		FileReader fileReader = new FileReader(fileName);
		PushbackReader reader = new PushbackReader(fileReader);
		
		return read(reader);
	}
	
	/**
	 * @param in
	 * @return
	 * @throws Exception
	 */
	public static List<Object> read(PushbackReader in) throws Exception { 
		// find the start ... and move one past it.
		char c = (char) in.read();
		while (c == ';') {
			removeLine(in);
			c = (char) in.read();
		}
		
		while (in.ready() && c != '(')  
			c = (char) in.read();

		if (!in.ready())  
			return null;

		List<Object> results = new ArrayList<Object>();
		// remove any whitespace
		boolean keepRunning = true;
		while (keepRunning) {
			c = (char) in.read();
			while (Character.isWhitespace(c))
				c = (char) in.read();
		
			if (Character.isLetter(c)) { 
				in.unread(c);
				results.add(readSymbol(in));
			} else { 
				switch (c) { 
				case '(':
					in.unread(c);
					results.add(read(in));
					break;
				case '"':
					results.add(readString(in));
					break;
				case ')':
					keepRunning = false;
					break;
				default:
					in.unread(c);
					results.add(readNumber(in));
					break;
				}
			}
		}
		
		return results;
	}
	
	private static void removeLine(PushbackReader in) throws Exception { 
		char c = (char) in.read();
		while (c != '\n') { 
			c = (char) in.read();
		}
	}

}
