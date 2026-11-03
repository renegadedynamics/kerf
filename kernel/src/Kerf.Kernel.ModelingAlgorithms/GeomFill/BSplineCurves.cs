// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_BSplineCurves.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_BSplineCurves.lxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_BSplineCurves.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Geometry;
using Occt;
using static Kerf.Engine.Construction.Surfaces.FillingStyle;

namespace Kerf.Engine.Construction.Surfaces;

/// <summary>
/// A B-spline surface filling two, three or four contiguous B-spline curves, in the Stretch, Coons or Curved style.
/// The curves are arranged (and reversed if needed) to form the boundary; three curves are closed by a straight
/// segment, and two curves give a ruled surface (Stretch, Coons) or a translated one (Curved).
/// </summary>
/// <remarks>
/// A working object (decision 21): <c>Init</c> recomputes the surface. The caller's curves are copied, never
/// changed. <see cref="Surface"/> hands out the surface frozen; ported callers use <see cref="InternalSurface"/>.
/// </remarks>
public class BSplineCurves
{
    private BSplineSurface? _surface;

    // Arrange: prepares the filling. The curves are arranged in this way:
    //
    //                      CC3
    //                  ----->-----
    //                 |           |
    //                 |           |
    //                 |           |
    //             CC4 ^           ^ CC2
    //                 |           |
    //                 |           |
    //                  ----->-----
    //                   CC1 = C1
    //
    // A curve degenerated to a point that matches the last end point is taken first.
    private static bool Arrange(BSplineCurve3D c1, BSplineCurve3D c2, BSplineCurve3D c3, BSplineCurve3D c4,
                                out BSplineCurve3D cc1, out BSplineCurve3D cc2, out BSplineCurve3D cc3,
                                out BSplineCurve3D cc4, double tol)
    {
        BSplineCurve3D[] gc = new BSplineCurve3D[4];
        BSplineCurve3D dummy;
        gc[0] = (BSplineCurve3D)c1.Copy();
        gc[1] = (BSplineCurve3D)c2.Copy();
        gc[2] = (BSplineCurve3D)c3.Copy();
        gc[3] = (BSplineCurve3D)c4.Copy();
        cc1 = cc2 = cc3 = cc4 = null!;

        int i, j;
        bool trouve;

        for (i = 1; i <= 3; i++)
        {
            trouve = false;
            // search for a degenerated curve = point, which would match first
            for (j = i; j <= 3 && !trouve; j++)
            {
                if (gc[j].StartPoint.DistanceTo(gc[j].EndPoint) < tol)
                {
                    // this is a degenerated line, does it match the last endpoint?
                    if (gc[j].StartPoint.DistanceTo(gc[i - 1].EndPoint) < tol)
                    {
                        dummy = gc[i];
                        gc[i] = gc[j];
                        gc[j] = dummy;
                        trouve = true;
                    }
                }
            }
            // if no degenerated curve matched, try an ordinary one as next curve
            if (!trouve)
            {
                for (j = i; j <= 3 && !trouve; j++)
                {
                    if (gc[j].StartPoint.DistanceTo(gc[i - 1].EndPoint) < tol)
                    {
                        dummy = gc[i];
                        gc[i] = gc[j];
                        gc[j] = dummy;
                        trouve = true;
                    }
                    else if (gc[j].EndPoint.DistanceTo(gc[i - 1].EndPoint) < tol)
                    {
                        gc[j] = (BSplineCurve3D)gc[j].Reversed();
                        dummy = gc[i];
                        gc[i] = gc[j];
                        gc[j] = dummy;
                        trouve = true;
                    }
                }
            }
            // if still non matched -> error, the algorithm cannot finish
            if (!trouve)
            {
                return false;
            }
        }

        cc1 = gc[0];
        cc2 = gc[1];
        cc3 = (BSplineCurve3D)gc[2].Reversed();
        cc4 = (BSplineCurve3D)gc[3].Reversed();

        return true;
    }

    private static int SetSameDistribution(ref BSplineCurve3D c1, ref BSplineCurve3D c2)
    {
        NCollection_Array1<Point3D> p1 = new(c1.InternalPoles());
        NCollection_Array1<double> w1 = c1.InternalWeightsArray();
        NCollection_Array1<double> k1 = new(c1.InternalKnots());
        NCollection_Array1<int> m1 = new(c1.InternalMultiplicities());
        NCollection_Array1<Point3D> p2 = new(c2.InternalPoles());
        NCollection_Array1<double> w2 = c2.InternalWeightsArray();
        NCollection_Array1<double> k2 = new(c2.InternalKnots());
        NCollection_Array1<int> m2 = new(c2.InternalMultiplicities());

        double k11 = k1[1];
        double k12 = k1[k1.Upper()];
        double k21 = k2[1];
        double k22 = k2[k2.Upper()];

        if ((k12 - k11) > (k22 - k21))
        {
            global::Kerf.Engine.Geometry.Splines.BSplCLib.Reparametrize(k11, k12, k2);
            c2.SetKnots(k2);
        }
        else if ((k12 - k11) < (k22 - k21))
        {
            global::Kerf.Engine.Geometry.Splines.BSplCLib.Reparametrize(k21, k22, k1);
            c1.SetKnots(k1);
        }
        else if (Math.Abs(k12 - k11) > global::Kerf.Engine.Foundation.Precision.PConfusion())
        {
            global::Kerf.Engine.Geometry.Splines.BSplCLib.Reparametrize(k11, k12, k2);
            c2.SetKnots(k2);
        }

        int np, nk;
        if (global::Kerf.Engine.Geometry.Splines.BSplCLib.PrepareInsertKnots(c1.Degree, false, k1, m1, k2, m2, out np, out nk, global::Kerf.Engine.Foundation.Precision.PConfusion(), false))
        {
            NCollection_Array1<Point3D> newP = new(1, np);
            NCollection_Array1<double> newW = new(1, np);
            NCollection_Array1<double> newK = new(1, nk);
            NCollection_Array1<int> newM = new(1, nk);
            global::Kerf.Engine.Geometry.Splines.BSplCLib.InsertKnots(c1.Degree, false, p1, w1, k1, m1, k2, m2, newP, newW, newK, newM,
                                 global::Kerf.Engine.Foundation.Precision.PConfusion(), false);
            if (c1.IsRational)
            {
                c1 = new BSplineCurve3D(newP, newW, newK, newM, c1.Degree);
            }
            else
            {
                c1 = new BSplineCurve3D(newP, newK, newM, c1.Degree);
            }
            global::Kerf.Engine.Geometry.Splines.BSplCLib.InsertKnots(c2.Degree, false, p2, w2, k2, m2, k1, m1, newP, newW, newK, newM,
                                 global::Kerf.Engine.Foundation.Precision.PConfusion(), false);
            if (c2.IsRational)
            {
                c2 = new BSplineCurve3D(newP, newW, newK, newM, c2.Degree);
            }
            else
            {
                c2 = new BSplineCurve3D(newP, newK, newM, c2.Degree);
            }
        }
        else
        {
            throw new ConstructionErrorException(" ");
        }

        return c1.PoleCount;
    }

    public BSplineCurves()
    {
    }

    /// <summary>The filling of four contiguous curves in the style <paramref name="type"/>.</summary>
    public BSplineCurves(BSplineCurve3D c1, BSplineCurve3D c2, BSplineCurve3D c3,
                                  BSplineCurve3D c4, FillingStyle type)
    {
        Init(c1, c2, c3, c4, type);
    }

    /// <summary>The filling of three contiguous curves, closed by a segment, in the style <paramref name="type"/>.</summary>
    public BSplineCurves(BSplineCurve3D c1, BSplineCurve3D c2, BSplineCurve3D c3,
                                  FillingStyle type)
    {
        Init(c1, c2, c3, type);
    }

    /// <summary>The filling between two curves in the style <paramref name="type"/>.</summary>
    public BSplineCurves(BSplineCurve3D c1, BSplineCurve3D c2, FillingStyle type)
    {
        Init(c1, c2, type);
    }

    /// <summary>
    /// Fills four contiguous curves. Throws Standard_ConstructionError if they don't join, or for the Coons style
    /// with fewer than four poles on a side.
    /// </summary>
    public void Init(BSplineCurve3D c1, BSplineCurve3D c2, BSplineCurve3D c3, BSplineCurve3D c4,
                     FillingStyle type)
    {
        // On ordonne les courbes
        BSplineCurve3D cc1, cc2, cc3, cc4;

        double tol = global::Kerf.Engine.Foundation.Precision.Confusion();
        bool isOK = Arrange(c1, c2, c3, c4, out cc1, out cc2, out cc3, out cc4, tol);

        Standard_ConstructionError_Raise_if(!isOK, " GeomFill_BSplineCurves: Courbes non jointives");

        // Mise en conformite des degres
        int deg1 = cc1.Degree;
        int deg2 = cc2.Degree;
        int deg3 = cc3.Degree;
        int deg4 = cc4.Degree;
        int degU = Max(deg1, deg3);
        int degV = Max(deg2, deg4);
        if (deg1 < degU)
        {
            cc1.IncreaseDegree(degU);
        }
        if (deg2 < degV)
        {
            cc2.IncreaseDegree(degV);
        }
        if (deg3 < degU)
        {
            cc3.IncreaseDegree(degU);
        }
        if (deg4 < degV)
        {
            cc4.IncreaseDegree(degV);
        }

        // Mise en conformite des distributions de noeuds
        int nbUPoles = SetSameDistribution(ref cc1, ref cc3);
        int nbVPoles = SetSameDistribution(ref cc2, ref cc4);

        if (type == FillingStyle.CoonsStyle)
        {
            if (nbUPoles < 4 || nbVPoles < 4)
            {
                throw new ConstructionErrorException("GeomFill_BSplineCurves: invalid filling style");
            }
        }

        NCollection_Array1<Point3D> p1 = cc1.InternalPoles();
        NCollection_Array1<Point3D> p2 = cc2.InternalPoles();
        NCollection_Array1<Point3D> p3 = cc3.InternalPoles();
        NCollection_Array1<Point3D> p4 = cc4.InternalPoles();

        // Traitement des courbes rationelles
        bool isRat = (cc1.IsRational || cc2.IsRational || cc3.IsRational || cc4.IsRational);

        NCollection_Array1<double> w1 = cc1.InternalWeightsArray();
        NCollection_Array1<double> w2 = cc2.InternalWeightsArray();
        NCollection_Array1<double> w3 = cc3.InternalWeightsArray();
        NCollection_Array1<double> w4 = cc4.InternalWeightsArray();

        Filling caro = new();
        if (isRat)
        {
            switch (type)
            {
                case FillingStyle.StretchStyle:
                    caro = new Stretch(p1, p2, p3, p4, w1, w2, w3, w4);
                    break;
                case FillingStyle.CoonsStyle:
                    caro = new Coons(p1, p4, p3, p2, w1, w4, w3, w2);
                    break;
                case FillingStyle.CurvedStyle:
                    caro = new Curved(p1, p2, p3, p4, w1, w2, w3, w4);
                    break;
            }
        }
        else
        {
            switch (type)
            {
                case FillingStyle.StretchStyle:
                    caro = new Stretch(p1, p2, p3, p4);
                    break;
                case FillingStyle.CoonsStyle:
                    caro = new Coons(p1, p4, p3, p2);
                    break;
                case FillingStyle.CurvedStyle:
                    caro = new Curved(p1, p2, p3, p4);
                    break;
            }
        }

        nbUPoles = caro.UPoleCount;
        nbVPoles = caro.VPoleCount;
        NCollection_Array2<Point3D> poles = new(1, nbUPoles, 1, nbVPoles);

        // Creation de la surface
        NCollection_Array1<double> uKnots = cc1.InternalKnots();
        NCollection_Array1<int> uMults = cc1.InternalMultiplicities();
        NCollection_Array1<double> vKnots = cc2.InternalKnots();
        NCollection_Array1<int> vMults = cc2.InternalMultiplicities();

        caro.Poles(poles);

        if (caro.IsRational())
        {
            NCollection_Array2<double> weights = new(1, nbUPoles, 1, nbVPoles);
            caro.Weights(weights);
            _surface = new BSplineSurface(poles, weights, uKnots, vKnots, uMults, vMults, cc1.Degree,
                                                cc2.Degree);
        }
        else
        {
            _surface = new BSplineSurface(poles, uKnots, vKnots, uMults, vMults, cc1.Degree, cc2.Degree);
        }
    }

    /// <summary>Fills three contiguous curves, the fourth side being the segment between the free ends.</summary>
    public void Init(BSplineCurve3D c1, BSplineCurve3D c2, BSplineCurve3D c3, FillingStyle type)
    {
        BSplineCurve3D c4;
        NCollection_Array1<Point3D> poles = new(1, 2);
        NCollection_Array1<double> knots = new(1, 2);
        NCollection_Array1<int> mults = new(1, 2);
        double tol = global::Kerf.Engine.Foundation.Precision.Confusion();
        tol = tol * tol;
        if (c1.StartPoint.SquareDistanceTo(c2.StartPoint) > tol
            && c1.StartPoint.SquareDistanceTo(c2.EndPoint) > tol)
        {
            poles[1] = c1.StartPoint;
        }
        else
        {
            poles[1] = c1.EndPoint;
        }

        if (c3.StartPoint.SquareDistanceTo(c2.StartPoint) > tol
            && c3.StartPoint.SquareDistanceTo(c2.EndPoint) > tol)
        {
            poles[2] = c3.StartPoint;
        }
        else
        {
            poles[2] = c3.EndPoint;
        }

        knots[1] = c2.Knot(c2.FirstUKnotIndex());
        knots[2] = c2.Knot(c2.LastUKnotIndex());
        mults[1] = mults[2] = 2;
        c4 = new BSplineCurve3D(poles, knots, mults, 1);
        Init(c1, c2, c3, c4, type);
    }

    /// <summary>
    /// Fills between two curves: a ruled surface for the Stretch and Coons styles, the first curve translated along
    /// the second for the Curved style (which needs the curves to share an end; else Standard_OutOfRange).
    /// </summary>
    public void Init(BSplineCurve3D c1, BSplineCurve3D c2, FillingStyle type)
    {
        BSplineCurve3D cc1 = (BSplineCurve3D)c1.Copy();
        BSplineCurve3D cc2 = (BSplineCurve3D)c2.Copy();

        int deg1 = cc1.Degree;
        int deg2 = cc2.Degree;

        bool isRat = (cc1.IsRational || cc2.IsRational);

        if (type != FillingStyle.CurvedStyle)
        {
            int degU = Max(deg1, deg2);

            if (cc1.Degree < degU)
            {
                cc1.IncreaseDegree(degU);
            }
            if (cc2.Degree < degU)
            {
                cc2.IncreaseDegree(degU);
            }

            // Mise en conformite des distributions de noeuds
            int nbPoles = SetSameDistribution(ref cc1, ref cc2);
            NCollection_Array2<Point3D> poles = new(1, nbPoles, 1, 2);
            NCollection_Array1<Point3D> p1 = cc1.InternalPoles();
            NCollection_Array1<Point3D> p2 = cc2.InternalPoles();

            int i;
            for (i = 1; i <= nbPoles; i++)
            {
                poles[i, 1] = p1[i];
                poles[i, 2] = p2[i];
            }
            NCollection_Array1<double> uKnots = cc1.InternalKnots();
            NCollection_Array1<int> uMults = cc1.InternalMultiplicities();
            NCollection_Array1<double> vKnots = new(1, 2);
            NCollection_Array1<int> vMults = new(1, 2);
            vKnots[1] = 0;
            vKnots[2] = 1;
            vMults[1] = 2;
            vMults[2] = 2;

            // Traitement des courbes rationelles
            if (isRat)
            {
                NCollection_Array2<double> weights = new(1, nbPoles, 1, 2);
                NCollection_Array1<double> w1 = cc1.InternalWeightsArray();
                NCollection_Array1<double> w2 = cc2.InternalWeightsArray();
                for (i = 1; i <= nbPoles; i++)
                {
                    weights[i, 1] = w1[i];
                    weights[i, 2] = w2[i];
                }
                _surface = new BSplineSurface(poles, weights, uKnots, vKnots, uMults, vMults, cc1.Degree, 1,
                                                    cc1.IsPeriodic, false);
            }
            else
            {
                _surface = new BSplineSurface(poles, uKnots, vKnots, uMults, vMults, cc1.Degree, 1);
            }
        }
        else
        {
            double eps = global::Kerf.Engine.Foundation.Precision.Confusion();
            bool isOK = false;
            if (cc1.StartPoint.IsEqual(cc2.StartPoint, eps))
            {
                isOK = true;
            }
            else if (cc1.StartPoint.IsEqual(cc2.EndPoint, eps))
            {
                cc2.Reverse();
                isOK = true;
            }
            else if (cc1.EndPoint.IsEqual(cc2.StartPoint, eps))
            {
                // OCCT: reverses the caller's C1, not its copy CC1, so the filling is built from the unreversed
                // copy. The port leaves the caller's curve unchanged (decision 846): the result is OCCT's
                // (occt-issues.md).
                isOK = true;
            }
            else if (cc1.EndPoint.IsEqual(cc2.EndPoint, eps))
            {
                cc1.Reverse();
                cc2.Reverse();
                isOK = true;
            }

            if (!isOK)
            {
                throw new OutOfRangeException("GeomFill_BSplineCurves: Courbes non jointives");
            }

            int nbUPoles = cc1.PoleCount;
            int nbVPoles = cc2.PoleCount;
            NCollection_Array1<Point3D> p1 = cc1.InternalPoles();
            NCollection_Array1<Point3D> p2 = cc2.InternalPoles();

            NCollection_Array1<double> uKnots = cc1.InternalKnots();
            NCollection_Array1<int> uMults = cc1.InternalMultiplicities();
            NCollection_Array1<double> vKnots = cc2.InternalKnots();
            NCollection_Array1<int> vMults = cc2.InternalMultiplicities();

            Filling caro;
            if (isRat)
            {
                NCollection_Array1<double> w1 = cc1.InternalWeightsArray();
                NCollection_Array1<double> w2 = cc2.InternalWeightsArray();
                caro = new Curved(p1, p2, w1, w2);
            }
            else
            {
                caro = new Curved(p1, p2);
            }

            nbUPoles = caro.UPoleCount;
            nbVPoles = caro.VPoleCount;
            NCollection_Array2<Point3D> poles = new(1, nbUPoles, 1, nbVPoles);
            caro.Poles(poles);

            if (caro.IsRational())
            {
                NCollection_Array2<double> weights = new(1, nbUPoles, 1, nbVPoles);
                caro.Weights(weights);
                _surface = new BSplineSurface(poles, weights, uKnots, vKnots, uMults, vMults, deg1, deg2, false,
                                                    false);
            }
            else
            {
                _surface = new BSplineSurface(poles, uKnots, vKnots, uMults, vMults, deg1, deg2, false, false);
            }
        }
    }

    /// <summary>The surface (null before <c>Init</c>), frozen.</summary>
    public BSplineSurface? Surface()
    {
        _surface?.Freeze();
        return _surface;
    }

    /// <summary>The surface, not frozen, for ported callers.</summary>
    internal BSplineSurface? InternalSurface() => _surface;
}
