package br.ufsc.similarity.utils;

import br.ufsc.similarity.base.elliptical.ETrajectory;
import br.ufsc.similarity.base.elliptical.Ellipse;
import br.ufsc.similarity.base.raw.TPoint;
import br.ufsc.similarity.base.raw.Trajectory;
/**
 * 
 * @author Andre Salvaro Furtado
 *
 */
public class CreateEllipseMath {

	public static ETrajectory createEllipticalTrajectory(Trajectory t) {
		int i = 0;
		ETrajectory T = new ETrajectory(t.getTid());

		while (i < t.getPoints().size() - 1) {

			TPoint p1 = t.getPoint(i);
			TPoint p2 = t.getPoint(i + 1);

			double x = (p1.getX() + p2.getX()) / 2;
			double y = (p1.getY() + p2.getY()) / 2;

			double fociDistance = Distance.euclidean(p1, p2);
			double majorAxis = Distance.triangular(p1, p2) + 1;

			double fociDistanceSquare = fociDistance * fociDistance;
			double majorAxisSquare = majorAxis * majorAxis;

			double minorAxis = Math.sqrt(majorAxisSquare - fociDistanceSquare);

			double angleO = Distance.angle(p1, p2);

			Ellipse e = new Ellipse();
			e.setEid(i);
			e.setStartTime(p1.getTimestamp());
			e.setEndTime(p2.getTimestamp());
			e.setCenter(new TPoint(x, y));
			e.setF1(p1);
			e.setF2(p2);
			e.setSemiMajorAxis(majorAxis / 2);
			e.setMajorAxis(majorAxis);
			e.setMinorAxis(minorAxis);
			e.setAngle(angleO);
			e.setEccentricity(fociDistance);
			T.addEllipse(e);

			i++;
		}

		return T;
	}
}
