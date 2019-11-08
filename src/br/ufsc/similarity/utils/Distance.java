package br.ufsc.similarity.utils;

import br.ufsc.similarity.base.Point;
import br.ufsc.similarity.base.raw.TPoint;
/**
 * 
 * @author Andre Salvaro Furtado
 *
 */

public class Distance {
	
	public static double euclidean(Point p1,Point p2){
		double distX = Math.abs(p1.getX()-p2.getX());
		double distXSquare = distX*distX;
		
		double distY = Math.abs(p1.getY()-p2.getY());
		double distYSquare = distY*distY;
		
		return Math.sqrt(distXSquare+distYSquare);
	}
	
	public static double triangular(Point p1, Point p2){
		double distX = Math.abs(p1.getX()-p2.getX());
		double distXSquare = distX*distX;
		
		double distY = Math.abs(p1.getY()-p2.getY());
		double distYSquare = distY*distY;

		return Math.sqrt(2*(distXSquare+distYSquare));
	}
		
	public static double angle(TPoint p1,TPoint p2){
		return Math.atan2(p2.getX() - p1.getX(), p2.getY()-p1.getY());
	}	
}
