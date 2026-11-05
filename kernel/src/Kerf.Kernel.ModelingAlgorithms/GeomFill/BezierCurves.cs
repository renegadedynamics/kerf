// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_BezierCurves.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_BezierCurves.lxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_BezierCurves.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Geometry;
using Occt;
using static Kerf.Engine.Construction.Surfaces.FillingStyle;

namespace Kerf.Engine.Construction.Surfaces;

/// <summary>
/// A Bezier surface filling two, three or four contiguous Bezier curves, in the Stretch, Coons or Curved style.
/// The curves are arranged (and reversed if needed) to form the boundary; three curves are closed by a straight
/// segment, and two curves give a ruled surface (Stretch, Coons) or a translated one (Curved).
/// </summary>
/// <remarks>
/// A working object (decision 21): <c>Init</c> recomputes the surface. The caller's curves are copied, never
/// changed. <see cref="Surface"/> hands out the surface frozen; ported callers use <see cref="InternalSurface"/>.
/// </remarks>
internal class BezierCurves
{
    private BezierSurface? _surface;

    // SetSameWeights uses the following property of rational Bezier curves: if Wi = Weight(i), Pi = Pole(i),
    // n = NbPoles, then for any a, b, c != 0 the transformation Wi = a * b^i * c^(n-i) doesn't modify the geometry
    // of the curve; only the lengths of the derivatives change.
    private static void SetSameWeights(NCollection_Array1<double> w1, NCollection_Array1<double> w2,
                                       NCollection_Array1<double> w3, NCollection_Array1<double> w4)
    {
        double eps = global::Kerf.Engine.Foundation.Precision.Confusion();

        int nu = w1.Length();
        int nv = w2.Length();

        double a = (w1[1] * w2[1]) / (w1[nu] * w2[nv]);
        double b = (w3[1] * w4[1]) / (w3[nu] * w4[nv]);

        int i;
        double alfa = w1[nu] / w2[1];
        for (i = 1; i <= nv; i++)
        {
            w2[i] *= alfa;
        }
        double beta = w2[nv] / w3[nu];
        for (i = 1; i <= nu; i++)
        {
            w3[i] *= beta;
        }
        double gamma = w3[1] / w4[nv];
        for (i = 1; i <= nv; i++)
        {
            w4[i] *= gamma;
        }

        if (Math.Abs(a - b) > eps)
        {
            double w = Math.Pow(w1[1] / w4[1], 1.0 / (double)(nv - 1));
            double x = w;
            for (i = nv - 1; i >= 1; i--)
            {
                w4[i] *= x;
                x *= w;
            }
        }
    }

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
    private static bool Arrange(BezierCurve3D c1, BezierCurve3D c2, BezierCurve3D c3, BezierCurve3D c4,
                                out BezierCurve3D cc1, out BezierCurve3D cc2, out BezierCurve3D cc3,
                                out BezierCurve3D cc4, double tol)
    {
        BezierCurve3D[] gc = new BezierCurve3D[4];
        BezierCurve3D dummy;
        gc[0] = (BezierCurve3D)c1.Copy();
        gc[1] = (BezierCurve3D)c2.Copy();
        gc[2] = (BezierCurve3D)c3.Copy();
        gc[3] = (BezierCurve3D)c4.Copy();
        cc1 = cc2 = cc3 = cc4 = null!;

        int i, j;
        bool trouve;

        for (i = 1; i <= 3; i++)
        {
            trouve = false;
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
                    gc[j] = (BezierCurve3D)gc[j].Reversed();
                    dummy = gc[i];
                    gc[i] = gc[j];
                    gc[j] = dummy;
                    trouve = true;
                }
            }
            if (!trouve)
            {
                return false;
            }
        }

        cc1 = gc[0];
        cc2 = gc[1];
        cc3 = (BezierCurve3D)gc[2].Reversed();
        cc4 = (BezierCurve3D)gc[3].Reversed();

        return true;
    }

    // OCCT's Poles() and WeightsArray() const references: copies of the values (the curves are this object's own).
    private static NCollection_Array1<Point3D> PolesOf(BezierCurve3D c)
    {
        NCollection_Array1<Point3D> p = new(1, c.PoleCount);
        c.CopyPolesTo(p);
        return p;
    }

    private static NCollection_Array1<double> WeightsOf(BezierCurve3D c)
    {
        NCollection_Array1<double> w = new(1, c.PoleCount);
        c.CopyWeightsTo(w);
        return w;
    }

    public BezierCurves()
    {
    }

    /// <summary>The filling of four contiguous curves in the style <paramref name="type"/>.</summary>
    public BezierCurves(BezierCurve3D c1, BezierCurve3D c2, BezierCurve3D c3, BezierCurve3D c4,
                                 FillingStyle type)
    {
        Init(c1, c2, c3, c4, type);
    }

    /// <summary>The filling of three contiguous curves, closed by a segment, in the style <paramref name="type"/>.</summary>
    public BezierCurves(BezierCurve3D c1, BezierCurve3D c2, BezierCurve3D c3,
                                 FillingStyle type)
    {
        Init(c1, c2, c3, type);
    }

    /// <summary>The filling between two curves in the style <paramref name="type"/>.</summary>
    public BezierCurves(BezierCurve3D c1, BezierCurve3D c2, FillingStyle type)
    {
        Init(c1, c2, type);
    }

    /// <summary>Fills four contiguous curves; throws Standard_ConstructionError if they don't join.</summary>
    public void Init(BezierCurve3D c1, BezierCurve3D c2, BezierCurve3D c3, BezierCurve3D c4,
                     FillingStyle type)
    {
        // On ordonne les courbes
        BezierCurve3D cc1, cc2, cc3, cc4;

        double tol = global::Kerf.Engine.Foundation.Precision.Confusion();
        // OCCT's release build ignores the result (No_Exception); the port checks it (decision 9).
        bool isOK = Arrange(c1, c2, c3, c4, out cc1, out cc2, out cc3, out cc4, tol);

        Standard_ConstructionError_Raise_if(!isOK, " GeomFill_BezierCurves: Courbes non jointives");

        // Mise en conformite des degres
        int degU = Max(cc1.Degree, cc3.Degree);
        int degV = Max(cc2.Degree, cc4.Degree);

        if (type == FillingStyle.CoonsStyle)
        {
            degU = Max(degU, 3);
            degV = Max(degV, 3);
        }

        if (cc1.Degree < degU)
        {
            cc1.Increase(degU);
        }
        if (cc2.Degree < degV)
        {
            cc2.Increase(degV);
        }
        if (cc3.Degree < degU)
        {
            cc3.Increase(degU);
        }
        if (cc4.Degree < degV)
        {
            cc4.Increase(degV);
        }

        NCollection_Array1<Point3D> p1 = PolesOf(cc1);
        NCollection_Array1<Point3D> p2 = PolesOf(cc2);
        NCollection_Array1<Point3D> p3 = PolesOf(cc3);
        NCollection_Array1<Point3D> p4 = PolesOf(cc4);

        // Traitement des courbes rationelles
        bool isRat = (cc1.IsRational || cc2.IsRational || cc3.IsRational || cc4.IsRational);

        Filling caro = new();
        if (isRat)
        {
            NCollection_Array1<double> w1 = WeightsOf(cc1);
            NCollection_Array1<double> w2 = WeightsOf(cc2);
            NCollection_Array1<double> w3 = WeightsOf(cc3);
            NCollection_Array1<double> w4 = WeightsOf(cc4);

            // Mise en conformite des poids aux coins.
            SetSameWeights(w1, w2, w3, w4);

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

        int nbUPoles = caro.UPoleCount;
        int nbVPoles = caro.VPoleCount;
        NCollection_Array2<Point3D> poles = new(1, nbUPoles, 1, nbVPoles);
        caro.Poles(poles);

        if (caro.IsRational())
        {
            NCollection_Array2<double> weights = new(1, nbUPoles, 1, nbVPoles);
            caro.Weights(weights);
            _surface = new BezierSurface(poles, weights);
        }
        else
        {
            _surface = new BezierSurface(poles);
        }
    }

    /// <summary>Fills three contiguous curves, the fourth side being the segment between the free ends.</summary>
    public void Init(BezierCurve3D c1, BezierCurve3D c2, BezierCurve3D c3, FillingStyle type)
    {
        BezierCurve3D c4;
        NCollection_Array1<Point3D> poles = new(1, 2);
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
        c4 = new BezierCurve3D(poles);
        Init(c1, c2, c3, c4, type);
    }

    /// <summary>
    /// Fills between two curves: a ruled surface for the Stretch and Coons styles, the first curve translated along
    /// the second for the Curved style (which needs the curves to share an end; else Standard_OutOfRange).
    /// </summary>
    public void Init(BezierCurve3D c1, BezierCurve3D c2, FillingStyle type)
    {
        BezierCurve3D cc1 = (BezierCurve3D)c1.Copy();
        BezierCurve3D cc2 = (BezierCurve3D)c2.Copy();

        int deg1 = cc1.Degree;
        int deg2 = cc2.Degree;

        bool isRat = (cc1.IsRational || cc2.IsRational);

        if (type != FillingStyle.CurvedStyle)
        {
            int degU = Max(deg1, deg2);

            if (cc1.Degree < degU)
            {
                cc1.Increase(degU);
            }
            if (cc2.Degree < degU)
            {
                cc2.Increase(degU);
            }

            NCollection_Array2<Point3D> poles = new(1, degU + 1, 1, 2);
            NCollection_Array1<Point3D> p1 = PolesOf(cc1);
            NCollection_Array1<Point3D> p2 = PolesOf(cc2);

            int i;
            for (i = 1; i <= degU + 1; i++)
            {
                poles[i, 1] = p1[i];
                poles[i, 2] = p2[i];
            }
            if (isRat)
            {
                NCollection_Array1<double> w1 = WeightsOf(cc1);
                NCollection_Array1<double> w2 = WeightsOf(cc2);
                NCollection_Array2<double> weights = new(1, degU + 1, 1, 2);
                for (i = 1; i <= degU + 1; i++)
                {
                    weights[i, 1] = w1[i];
                    weights[i, 2] = w2[i];
                }
                _surface = new BezierSurface(poles, weights);
            }
            else
            {
                _surface = new BezierSurface(poles);
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
                throw new OutOfRangeException("GeomFill_BezierCurves: Courbes non jointives");
            }

            NCollection_Array1<Point3D> p1 = PolesOf(cc1);
            NCollection_Array1<Point3D> p2 = PolesOf(cc2);

            Filling caro;
            if (isRat)
            {
                NCollection_Array1<double> w1 = WeightsOf(cc1);
                NCollection_Array1<double> w2 = WeightsOf(cc2);
                caro = new Curved(p1, p2, w1, w2);
            }
            else
            {
                caro = new Curved(p1, p2);
            }

            int nbUPoles = caro.UPoleCount;
            int nbVPoles = caro.VPoleCount;
            NCollection_Array2<Point3D> poles = new(1, nbUPoles, 1, nbVPoles);
            caro.Poles(poles);

            if (caro.IsRational())
            {
                NCollection_Array2<double> weights = new(1, nbUPoles, 1, nbVPoles);
                caro.Weights(weights);
                _surface = new BezierSurface(poles, weights);
            }
            else
            {
                _surface = new BezierSurface(poles);
            }
        }
    }

    /// <summary>The surface (null before <c>Init</c>), frozen.</summary>
    public BezierSurface? Surface()
    {
        _surface?.Freeze();
        return _surface;
    }

    /// <summary>The surface, not frozen, for ported callers.</summary>
    internal BezierSurface? InternalSurface() => _surface;
}
