package br.ufsc.similarity.measure;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

import br.ufsc.similarity.base.TrajectorySimilarityCalculator;
import br.ufsc.similarity.base.elliptical.ETrajectory;
import br.ufsc.similarity.base.elliptical.Ellipse;
import br.ufsc.similarity.base.raw.TPoint;
import br.ufsc.similarity.base.raw.Trajectory;
import br.ufsc.similarity.utils.CreateEllipseMath;
import br.ufsc.similarity.utils.Distance;

/**
 * 
 * @author Andre Salvaro Furtado
 *
 */
public class UMS extends TrajectorySimilarityCalculator {
	public UMS() {

	}

	public double getDistance(Trajectory T1, Trajectory T2) {
		Set<Integer> aSet = new HashSet<>();
		Set<Integer> bSet = new HashSet<>();

		int n = T1.length();
		int m = T2.length();

		// At least 2 ellipses
		if (n < 3 || m < 3) {
			//XXX Exceptions for when the trajectories have less than 3 points, but the points are equal
			if (n == 1 && m == 1) {
				if (T1.getPoint(0).equals(T2.getPoint(0))) {
					return 1;
				}
			}
			if (n == 2 && m == 2) {
				if (T1.getPoint(0).equals(T2.getPoint(0)) && T1.getPoint(1).equals(T2.getPoint(1))) {
					return 1;
				}
			}
			return 0;
		}

		// Create Elliptical Trajectories (it can be performed offline speeding
		// up multiple similarity computations)
		ETrajectory E1 = CreateEllipseMath.createEllipticalTrajectory(T1);
		ETrajectory E2 = CreateEllipseMath.createEllipticalTrajectory(T2);

		// Compute Alikeness and Shareness
		List<Integer>[] aMatchSet = new ArrayList[n];
		List<Integer>[] bMatchSet = new ArrayList[m];

		double[] shr1 = new double[n];
		double[] shr2 = new double[m];

		for (int i = 0; i < n - 1; i++) {
			Ellipse e1 = E1.getEllipse(i);
			for (int j = 0; j < m - 1; j++) {

				Ellipse e2 = E2.getEllipse(j);
				if (Distance.euclidean(e1.getCenter(), e2.getCenter()) <= (e1.getMajorAxis() / 2)
						+ (e2.getMajorAxis() / 2)) {
					double p1shr = getShareness(e1.getF1(), e2);
					if (p1shr > 0) {
						aSet.add(j);
						if (aMatchSet[i] == null) {

							List<Integer> set = new ArrayList<Integer>();
							set.add(j);
							aMatchSet[i] = set;
						} else {
							aMatchSet[i].add(j);
						}
						if (shr1[i] != 1.0) {
							shr1[i] = p1shr > shr1[i] ? p1shr : shr1[i];
						}
					}

					double p2shr = getShareness(e1.getF2(), e2);
					if (p2shr > 0) {
						aSet.add(j);
						if (aMatchSet[i + 1] == null) {

							List<Integer> set = new ArrayList<Integer>();
							set.add(j);
							aMatchSet[i + 1] = set;
						} else {
							aMatchSet[i + 1].add(j);
						}
						if (shr1[i + 1] != 1.0) {
							shr1[i + 1] = p2shr > shr1[i + 1] ? p2shr : shr1[i + 1];
						}
					}

					double q1shr = getShareness(e2.getF1(), e1);
					if (q1shr > 0) {
						bSet.add(i);
						if (bMatchSet[j] == null) {
							List<Integer> set = new ArrayList<Integer>();
							set.add(i);
							bMatchSet[j] = set;
						} else {
							bMatchSet[j].add(i);
						}
						if (shr2[j] != 1.0) {
							shr2[j] = q1shr > shr2[j] ? q1shr : shr2[j];
						}
					}

					double q2shr = getShareness(e2.getF2(), e1);

					if (q2shr > 0) {
						bSet.add(i);
						if (bMatchSet[j + 1] == null) {
							List<Integer> set = new ArrayList<Integer>();
							set.add(i);
							bMatchSet[j + 1] = set;
						} else {
							bMatchSet[j + 1].add(i);
						}
						if (shr2[j + 1] != 1.0) {
							shr2[j + 1] = q2shr > shr2[j + 1] ? q2shr : shr2[j + 1];

						}
					}

				}
			}
		}

		// Compute Continuity
		int aContinuity[] = new int[n];
		int bContinuity[] = new int[m];

		double aResult = 0;
		double bResult = 0;

		for (int j = 0; j < n; j++) {
			List<Integer> matchingSet = aMatchSet[j];
			if (j == 0) {
				aContinuity[j] = matchingSet == null ? -1 : Collections.min(aMatchSet[j]);
			} else {
				aContinuity[j] = matchingSet == null ? -1 : getContinuityValue(aContinuity[j - 1], matchingSet);
			}
			if (aContinuity[j] != -1) {
				if (j == 0) {
					aResult++;
				} else if (aContinuity[j] >= aContinuity[j - 1]) {
					aResult++;
				}
			}
		}

		for (int j = 0; j < m; j++) {
			List<Integer> matchingSet = bMatchSet[j];
			if (j == 0) {

				bContinuity[j] = matchingSet == null ? -1 : Collections.min(bMatchSet[j]);
			} else {
				bContinuity[j] = matchingSet == null ? -1 : getContinuityValue(bContinuity[j - 1], matchingSet);
			}

			if (bContinuity[j] != -1) {
				if (j == 0) {
					bResult++;
				} else if (bContinuity[j] >= bContinuity[j - 1]) {
					bResult++;
				}
			}
		}

		double continuity = (aResult / n) * (bResult / m);

		int sum1 = 0;
		double sum3 = 0;
		for (int j = 0; j < n; j++) {
			if (shr1[j] > 0.0) {
				sum1 += 1;
				sum3 += shr1[j];
			}
		}

		int sum2 = 0;
		double sum4 = 0;
		for (int j = 0; j < m; j++) {
			if (shr2[j] > 0.0) {
				sum2 += 1;
				sum4 += shr2[j];
			}
		}

		double alikeness1 = ((double) sum1 / n);
		double alikeness2 = ((double) sum2 / m);

		double shareness1 = sum3 / n;
		double shareness2 = sum4 / m;

		double alikeness = (alikeness1 * alikeness2);
		double shareness = 0.5 * (shareness1 + shareness2);

		double similarity = (0.5 * (alikeness + shareness)) * continuity;
		return similarity;
	}

	public static int getContinuityValue(double lastValue, List<Integer> matchingList) {
		Collections.sort(matchingList);
		for (Integer i : matchingList) {
			if (i >= lastValue) {
				return i;
			}
		}
		return -1;
	}

	private double getShareness(TPoint p1, Ellipse e) {

		TPoint center = e.getCenter();

		double angle = e.getAngle();
		double cos = Math.cos(angle); // Angle in radians
		double sin = Math.sin(angle);

		double semiMinorAxis = e.getMinorAxis() / 2;
		double semiMajorAxis = e.getMajorAxis() / 2;

		double semiMinorAxisSquare = semiMinorAxis * semiMinorAxis;
		double semiMajorAxisSquare = semiMajorAxis * semiMajorAxis;

		double cx = center.getX();
		double cy = center.getY();

		double px = p1.getX();
		double py = p1.getY();

		double aCos = (cos * (px - cx) - sin * (py - cy));
		double a = aCos * aCos;
		double bSin = (sin * (px - cx) + cos * (py - cy));
		double b = bSin * bSin;

		double eIn = (a / semiMinorAxisSquare) + (b / semiMajorAxisSquare);

		if (eIn > 1) {
			return 0.0;
		}

		double distF1 = Distance.euclidean(p1, e.getF1());
		double distF2 = Distance.euclidean(p1, e.getF2());
		double distC = Distance.euclidean(p1, center);

		double hypotenuse = (semiMinorAxisSquare + semiMajorAxisSquare) / e.getMajorAxis();
		double min = 1;
		min = (Math.min(Math.min(distF1, distF2), distC) / hypotenuse);
		if (min == 0) {
			return 1;
		}
		return 1 - min;
	}
}
