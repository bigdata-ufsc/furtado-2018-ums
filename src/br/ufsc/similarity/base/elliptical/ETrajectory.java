package br.ufsc.similarity.base.elliptical;

import java.util.ArrayList;
import java.util.List;
/**
 * 
 * @author Andre Salvaro Furtado
 *
 */
public class ETrajectory {
	private final int tid;
	private List<Ellipse> ellipses;
	
	public ETrajectory(int tid){
		this.tid = tid;
		this.ellipses=new ArrayList<Ellipse>();
	}
	
	public void addEllipse(Ellipse e){
		this.ellipses.add(e);
	}
	
	public Ellipse getEllipse(int index){
		return this.ellipses.get(index);
	}
	
	public List<Ellipse> getEllipses(){
		return this.ellipses;
	}

	
	public int length(){
		return this.ellipses.size();
	}

	public int getTid(){
		return this.tid;
	}
}
