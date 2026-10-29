// Port of: src/ModelingData/TKGeomBase/Geom2dConvert/Geom2dConvert_BSplineCurveToBezierCurve.hxx
// Port of: src/ModelingData/TKGeomBase/Geom2dConvert/Geom2dConvert_BSplineCurveToBezierCurve.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// Converts a 2D B-spline curve into adjacent Bezier arcs, one per knot span (W. Boehm, "Generating the Bezier points
/// of B-spline curves and surfaces", CAD 13(6), 1981). The constructor prepares a copy of the curve; Arc(i) builds
/// each arc.
/// </summary>
public sealed class Geom2dConvert_BSplineCurveToBezierCurve
{
    private readonly Geom2d_BSplineCurve myCurve;

    /// <summary>Prepares the conversion of the whole of BasisCurve.</summary>
    public Geom2dConvert_BSplineCurveToBezierCurve(Geom2d_BSplineCurve BasisCurve)
    {
        myCurve = (Geom2d_BSplineCurve)BasisCurve.Copy();
        // periodic curve can't be converted correctly by two main reasons:
        // last pole (equal to first one) is missing;
        // poles recomputation using default boor scheme is fails.
        if (myCurve.IsPeriodic())
        {
            myCurve.SetNotPeriodic();
        }

        double Uf = myCurve.FirstParameter();
        double Ul = myCurve.LastParameter();
        myCurve.Segment(Uf, Ul);
        myCurve.IncreaseMultiplicity(myCurve.FirstUKnotIndex(), myCurve.LastUKnotIndex(), myCurve.Degree());
    }

    /// <summary>
    /// Prepares the conversion of BasisCurve between U1 and U2. A bound within ParametricTolerance / 2 of a knot,
    /// beyond it, is moved onto the knot.
    /// </summary>
    /// <exception cref="Standard_DomainError">U2 - U1 is below ParametricTolerance.</exception>
    public Geom2dConvert_BSplineCurveToBezierCurve(Geom2d_BSplineCurve BasisCurve, double U1, double U2,
                                                   double ParametricTolerance)
    {
        if (U2 - U1 < ParametricTolerance)
        {
            throw new Standard_DomainError("GeomConvert_BSplineCurveToBezierSurface");
        }

        double Uf = U1, Ul = U2;
        double PTol = ParametricTolerance / 2;

        int I1, I2;
        myCurve = (Geom2d_BSplineCurve)BasisCurve.Copy();
        if (myCurve.IsPeriodic())
        {
            myCurve.SetNotPeriodic();
        }

        myCurve.LocateU(U1, PTol, out I1, out I2);
        if (I1 == I2)
        { // We are on the knot
            if (myCurve.Knot(I1) > U1)
            {
                Uf = myCurve.Knot(I1);
            }
        }

        myCurve.LocateU(U2, PTol, out I1, out I2);
        if (I1 == I2)
        { // We are on the knot
            if (myCurve.Knot(I1) < U2)
            {
                Ul = myCurve.Knot(I1);
            }
        }

        myCurve.Segment(Uf, Ul);
        myCurve.IncreaseMultiplicity(myCurve.FirstUKnotIndex(), myCurve.LastUKnotIndex(), myCurve.Degree());
    }

    /// <summary>A new Bezier curve for arc Index (1 to NbArcs()), oriented as the B-spline.</summary>
    /// <exception cref="Standard_OutOfRange">Index is out of range.</exception>
    public Geom2d_BezierCurve Arc(int Index)
    {
        if (Index < 1 || Index > myCurve.NbKnots() - 1)
        {
            throw new Standard_OutOfRange("Geom2dConvert_BSplineCurveToBezierCurve");
        }

        int Deg = myCurve.Degree();

        NCollection_Array1<gp_Pnt2d> Poles = new(1, Deg + 1);

        Geom2d_BezierCurve C;
        if (myCurve.IsRational())
        {
            NCollection_Array1<double> Weights = new(1, Deg + 1);
            for (int i = 1; i <= Deg + 1; i++)
            {
                Poles[i] = myCurve.Pole(i + Deg * (Index - 1));
                Weights[i] = myCurve.Weight(i + Deg * (Index - 1));
            }

            C = new Geom2d_BezierCurve(Poles, Weights);
        }
        else
        {
            for (int i = 1; i <= Deg + 1; i++)
            {
                Poles[i] = myCurve.Pole(i + Deg * (Index - 1));
            }

            C = new Geom2d_BezierCurve(Poles);
        }

        return C;
    }

    /// <summary>
    /// Fills Curves, indexed from 1 to NbArcs() (OCCT documents a Standard_DimensionError for other bounds but
    /// doesn't check them), with new Bezier curves for all the arcs.
    /// </summary>
    public void Arcs(NCollection_Array1<Geom2d_BezierCurve> Curves)
    {
        int n = NbArcs();
        for (int i = 1; i <= n; i++)
        {
            Curves[i] = Arc(i);
        }
    }

    /// <summary>Copies the knots bounding the arcs (NbArcs() + 1 values) into TKnots from its lower bound.</summary>
    public void Knots(NCollection_Array1<double> TKnots)
    {
        int ii, kk;
        for (ii = 1, kk = TKnots.Lower(); ii <= myCurve.NbKnots(); ii++, kk++)
        {
            TKnots[kk] = myCurve.Knot(ii);
        }
    }

    /// <summary>The number of Bezier arcs.</summary>
    public int NbArcs() => (myCurve.NbKnots() - 1);
}
