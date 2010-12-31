package edu.arizona.cs.learn.util;

public class Pair<A, B> {
    private final A _first;
    private final B _second;

    public Pair(A first, B second) {
        super();
        _first = first;
        _second = second;
    }

    public int hashCode() {
        int hashFirst = _first != null ? _first.hashCode() : 0;
        int hashSecond = _second != null ? _second.hashCode() : 0;

        return (hashFirst + hashSecond) * hashSecond + hashFirst;
    }

    public boolean equals(Object other) {
        if (other instanceof Pair) {
                Pair otherPair = (Pair) other;
                return 
                ((  _first == otherPair._first ||
                        ( _first != null && otherPair._first != null &&
                          _first.equals(otherPair._first))) &&
                 (      _second == otherPair._second ||
                        ( _second != null && otherPair._second != null &&
                          _second.equals(otherPair._second))) );
        }

        return false;
    }

    public String toString()
    { 
           return "(" + _first + ", " + _second + ")"; 
    }

    public A getFirst() {
        return _first;
    }

    public B getSecond() {
        return _second;
    }
}