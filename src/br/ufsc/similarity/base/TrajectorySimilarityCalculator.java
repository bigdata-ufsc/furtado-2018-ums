package br.ufsc.similarity.base;

import br.ufsc.similarity.base.raw.Trajectory;

public abstract class TrajectorySimilarityCalculator {
	public abstract double getDistance(Trajectory t1,Trajectory t2);
}
