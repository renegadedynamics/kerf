// Port of: src/ModelingAlgorithms/TKShHealing/ShapeAnalysis/ShapeAnalysis_Geom.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeAnalysis/ShapeAnalysis_Geom.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;

using Kerf.Engine.Measurement;
using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Healing.Analysis;

/// <summary>Analysis of geometric entities: the plane nearest a set of points, a transformation from a 3x4 matrix.</summary>
public static class GeomAnalysis
{
    /// <summary>
    /// Builds a plane out of a set of points. Returns in <paramref name="maxDist"/> the largest distance between the
    /// plane and the points. <paramref name="pln"/> is written only when the result is true.
    /// </summary>
    public static bool NearestPlane(NCollection_Array1<Point3D> pnts, ref Plane3D pln, out double maxDist)
    {
        PEquation eq = new(pnts, 0.0);
        if (eq.GetType() == PEquation.Type.None)
        {
            maxDist = 0.0; // OCCT.NET: OCCT leaves the caller's value unchanged here
            return false;
        }

        double dev1 = eq.Extent(1);
        double dev2 = eq.Extent(2);
        double dev3 = eq.Extent(3);

        // Find the axis with the smallest extent - that's the plane normal candidate
        int axis = (dev1 < dev2) ? ((dev1 < dev3) ? 1 : 3) : ((dev2 < dev3) ? 2 : 3);

        // Check if the smallest extent is significantly smaller than the other two
        switch (axis)
        {
            case 1:
            {
                if ((2.0 * dev1 > dev2) || (2.0 * dev1 > dev3))
                {
                    axis = 0;
                }
                else
                {
                    pln = new Plane3D(eq.Barycentre(), new Direction3D(eq.PrincipalAxis(1)));
                }
                break;
            }
            case 2:
            {
                if ((2.0 * dev2 > dev1) || (2.0 * dev2 > dev3))
                {
                    axis = 0;
                }
                else
                {
                    pln = new Plane3D(eq.Barycentre(), new Direction3D(eq.PrincipalAxis(2)));
                }
                break;
            }
            case 3:
            {
                if ((2.0 * dev3 > dev2) || (2.0 * dev3 > dev1))
                {
                    axis = 0;
                }
                else
                {
                    pln = new Plane3D(eq.Barycentre(), new Direction3D(eq.PrincipalAxis(3)));
                }
                break;
            }
        }

        maxDist = RealFirst();
        if (axis != 0)
        {
            for (int i = pnts.Lower(); i <= pnts.Upper(); ++i)
            {
                double d = pln.DistanceTo(pnts[i]);
                if (maxDist < d)
                {
                    maxDist = d;
                }
            }
        }

        return (axis != 0);
    }

    /// <summary>
    /// Builds a transformation out of a 3 x 4 matrix (rows 1..3, columns 1..4); <paramref name="unit"/> multiplies the
    /// translation. False if the vectorial part isn't a similarity within <paramref name="prec"/>. A null matrix gives
    /// the identity.
    /// </summary>
    public static bool PositionTrsf(NCollection_HArray2<double>? coefs, out Transform3D trsf, double unit, double prec)
    {
        bool result = true;

        trsf = new Transform3D();

        if (coefs is null)
        {
            return true;
        }

        GeneralTransform3D gtrsf = new();
        for (int i = 1; i <= 3; i++)
        {
            for (int j = 1; j <= 4; j++)
            {
                gtrsf = gtrsf.WithValue(i, j, coefs.Value(i, j));
            }
        }

        Xyz v1 = new(gtrsf.Value(1, 1), gtrsf.Value(2, 1), gtrsf.Value(3, 1));
        Xyz v2 = new(gtrsf.Value(1, 2), gtrsf.Value(2, 2), gtrsf.Value(3, 2));
        Xyz v3 = new(gtrsf.Value(1, 3), gtrsf.Value(2, 3), gtrsf.Value(3, 3));
        double m1 = v1.Modulus();
        double m2 = v2.Modulus();
        double m3 = v3.Modulus();

        if (m1 < prec || m2 < prec || m3 < prec)
        {
            return false;
        }
        double mm  = (m1 + m2 + m3) / 3.0;
        double pmm = prec * mm;
        if (Math.Abs(m1 - mm) > pmm || Math.Abs(m2 - mm) > pmm || Math.Abs(m3 - mm) > pmm)
        {
            return false;
        }
        v1.Divide(m1);
        v2.Divide(m2);
        v3.Divide(m3);
        if (Math.Abs(v1.Dot(v2)) > prec || Math.Abs(v2.Dot(v3)) > prec || Math.Abs(v3.Dot(v1)) > prec)
        {
            return false;
        }

        if (v1.X() != 1 || v1.Y() != 0 || v1.Z() != 0 || v2.X() != 0 || v2.Y() != 1 || v2.Z() != 0
            || v3.X() != 0 || v3.Y() != 0 || v3.Z() != 1)
        {
            Direction3D d1 = new(v1);
            Direction3D d2 = new(v2);
            Direction3D d3 = new(v3);
            CoordinateSystem axes = new(new Point3D(0, 0, 0), d3, d1);
            d3 = d3.Crossed(d1);
            if (d3.Dot(d2) < 0)
            {
                axes = axes.YReversed();
            }
            trsf = trsf.WithTransformation(axes);
        }

        if (Math.Abs(mm - 1.0) > prec)
        {
            trsf = trsf.WithScale(new Point3D(0, 0, 0), mm); // OCCT: SetScale resets the transformation, dropping the rotation set above (occt-issues.md)
        }
        Vector3D tp = new(gtrsf.TranslationXyz());
        if (unit != 1.0)
        {
            tp = tp.Multiplied(unit);
        }
        if (tp.X != 0 || tp.Y != 0 || tp.Z != 0)
        {
            trsf = trsf.WithTranslationPart(tp);
        }

        return result;
    }
}
