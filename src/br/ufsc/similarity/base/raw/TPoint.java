package br.ufsc.similarity.base.raw;

import java.sql.Timestamp;

import br.ufsc.similarity.base.Point;
/**
 * 
 * @author Andr� Salvaro Furtado
 *
 */
public class TPoint extends Point {
	
	private Timestamp t;
	private int gid;
	private int occupation;
	
	public TPoint(int gid, double x, double y, Timestamp time, double speed, double transfX, double transfY) {
		super(x,y);
		this.t = time;
		this.gid=gid;
	}
	
	public TPoint(int gid, double x, double y, Timestamp time, double transfX, double transfY) {
		super(x,y);
		this.t = time;
	//	this.transformedGeom = new Geom2D(transfX, transfY);
		//	System.out.println("gid = "+gid);
//		System.out.println("x = "+transfX+" y ="+transfY);
//		System.out.println(transformedGeom);
		this.gid=gid;
	}
	
	public TPoint(int gid,double x, double y, Timestamp time, double speed) {
		super(x,y);
		this.t = time;
//		this.speed=speed;

		this.gid=gid;
	}
	
	public TPoint(int gid,double x, double y, Timestamp time) {
		super(x,y);
		this.t = time;
		this.gid=gid;
	}
	
	public TPoint(int gid,double x, double y, Timestamp time,int occupation) {
		super(x,y);
		this.t = time;
		this.gid=gid;
		this.occupation=occupation;
	}
	
	public TPoint(double x, double y) {
		super(x,y);
	}
	
	public int getOccupation(){
		return this.occupation;
	}

	public double getX(){
		return this.x;
	}
	
	public double getY(){
		return this.y;
	}
	
	public long getTime(){
		return this.t.getTime();
	}
	
	public Timestamp getTimestamp(){
		return this.t;
	}
	
	public double[] getCoord(){
		return new double[] {this.x, this.y};
	}
	
	public String getWKT(){
		StringBuilder wkt = new StringBuilder();
		wkt.append("POINT (").append(this.x).append(" ").append(this.y).append(")");
		return wkt.toString();
		
	}
	
//	public Geom2D getTransformedGeom(){
//		return transformedGeom;
//	}
	
	@Override
	public String toString() {
		return this.x + " " + this.y;
	}
	
	@Override
	public boolean equals(Object obj) {
		if(obj == this)
            return true;
        if(obj == null || obj.getClass() != this.getClass())
            return false;
		
		TPoint p = (TPoint) obj;
		return this.x == p.x && this.y == p.y && this.t.equals(p.t);
	}
	
//	@Override
//	public int hashCode() {
//		return Double.valueOf(x).hashCode() ^ Double.valueOf(y).hashCode() ^ Double.valueOf(t.getTime()).hashCode();
//	}

	public int getGid() {
		return gid;
	}

	public void setGid(int gid) {
		this.gid = gid;
	}
	
//	public double getSpeed() {
//		return speed;
//	}
}
