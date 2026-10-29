#if HAS_ORACLE
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GeomAbs_Shape;

namespace OcctNet.Oracle.Tests.Geom;

/// <summary>
/// Geom_Circle as OCCT writes it (ElCLib's evaluators, Geom_Conic's reversal), standing in for the port's
/// Geom_Circle (unit 1A) as the basis curve of swept surfaces. Replace with Geom_Circle once it is merged.
/// </summary>
internal sealed class PortCircle : Geom_Conic
{
    private double radius;

    public PortCircle(in gp_Ax2 A2, double R)
    {
        radius = R;
        pos = A2;
    }

    public override Geom_Geometry Copy() => new PortCircle(pos, radius);

    public override bool IsClosed() => true;

    public override bool IsPeriodic() => true;

    public override double ReversedParameter(double U) => (2.0 * Math.PI - U);

    public override double Eccentricity() => 0.0;

    public override double FirstParameter() => 0.0;

    public override double LastParameter() => 2.0 * Math.PI;

    public override gp_Pnt EvalD0(double U) => ElCLib.CircleValue(U, pos, radius);

    public override ResD1 EvalD1(double U)
    {
        ResD1 aResult = new ResD1();
        ElCLib.CircleD1(U, pos, radius, out aResult.Point, out aResult.D1);
        return aResult;
    }

    public override ResD2 EvalD2(double U)
    {
        ResD2 aResult = new ResD2();
        ElCLib.CircleD2(U, pos, radius, out aResult.Point, out aResult.D1, out aResult.D2);
        return aResult;
    }

    public override ResD3 EvalD3(double U)
    {
        ResD3 aResult = new ResD3();
        ElCLib.CircleD3(U, pos, radius, out aResult.Point, out aResult.D1, out aResult.D2, out aResult.D3);
        return aResult;
    }

    public override gp_Vec EvalDN(double U, int N) => ElCLib.CircleDN(U, pos, radius, N);

    protected internal override void Transform(in gp_Trsf T)
    {
        CheckMutable();
        radius = radius * Math.Abs(T.ScaleFactor());
        pos.Transform(T);
    }
}

/// <summary>
/// A non-periodic Geom_BSplineCurve as OCCT 8 evaluates, reverses and transforms it (BSplCLib directly, the span
/// located on the knots with multiplicities, DN on the flat knots), standing in for the port's Geom_BSplineCurve
/// (unit 1A). Continuity and closure are not mirrored: the tests don't compare them for this curve.
/// </summary>
internal sealed class PortBSplineCurve : Geom_Curve
{
    private readonly NCollection_Array1<gp_Pnt> myPoles;
    private readonly NCollection_Array1<double>? myWeights;
    private readonly NCollection_Array1<double> myKnots;
    private readonly NCollection_Array1<int> myMults;
    private readonly NCollection_Array1<double> myFlatKnots;
    private readonly int myDeg;

    public PortBSplineCurve(NCollection_Array1<gp_Pnt> thePoles, NCollection_Array1<double>? theWeights,
                            NCollection_Array1<double> theKnots, NCollection_Array1<int> theMults, int theDegree)
    {
        myPoles = new NCollection_Array1<gp_Pnt>(thePoles);
        myWeights = theWeights is null ? null : new NCollection_Array1<double>(theWeights);
        myKnots = new NCollection_Array1<double>(theKnots);
        myMults = new NCollection_Array1<int>(theMults);
        myDeg = theDegree;
        myFlatKnots = new NCollection_Array1<double>(1, Occt.BSplCLib.KnotSequenceLength(myMults, myDeg, false));
        Occt.BSplCLib.KnotSequence(myKnots, myMults, myDeg, false, myFlatKnots);
    }

    public override Geom_Geometry Copy() => new PortBSplineCurve(myPoles, myWeights, myKnots, myMults, myDeg);

    protected internal override void Reverse()
    {
        CheckMutable();
        Occt.BSplCLib.Reverse(myKnots);
        Occt.BSplCLib.Reverse(myMults);
        int last = myPoles.Upper();
        Occt.BSplCLib.Reverse(myPoles, last);
        if (myWeights is not null)
        {
            Occt.BSplCLib.Reverse(myWeights, last);
        }
        Occt.BSplCLib.KnotSequence(myKnots, myMults, myDeg, false, myFlatKnots);
    }

    public override double ReversedParameter(double U) => (FirstParameter() + LastParameter() - U);

    public override double FirstParameter() => myFlatKnots[myDeg + 1];

    public override double LastParameter() => myFlatKnots[myFlatKnots.Upper() - myDeg];

    public override bool IsClosed() => false;

    public override bool IsPeriodic() => false;

    public override GeomAbs_Shape Continuity() => GeomAbs_C0;

    public override bool IsCN(int N) => N <= 0;

    private void Locate(double U, out int theSpan, out double theNewU)
    {
        theSpan = 0;
        Occt.BSplCLib.LocateParameter(myDeg, myKnots, myMults, U, false, ref theSpan, out theNewU);
        if (theNewU < myKnots[theSpan])
        {
            theSpan--;
        }
    }

    public override gp_Pnt EvalD0(double U)
    {
        Locate(U, out int aSpan, out double aNewU);
        Occt.BSplCLib.D0(aNewU, aSpan, myDeg, false, myPoles, myWeights, myKnots, myMults, out gp_Pnt P);
        return P;
    }

    public override ResD1 EvalD1(double U)
    {
        ResD1 aResult = new ResD1();
        Locate(U, out int aSpan, out double aNewU);
        Occt.BSplCLib.D1(aNewU, aSpan, myDeg, false, myPoles, myWeights, myKnots, myMults, out aResult.Point, out aResult.D1);
        return aResult;
    }

    public override ResD2 EvalD2(double U)
    {
        ResD2 aResult = new ResD2();
        Locate(U, out int aSpan, out double aNewU);
        Occt.BSplCLib.D2(aNewU, aSpan, myDeg, false, myPoles, myWeights, myKnots, myMults,
                    out aResult.Point, out aResult.D1, out aResult.D2);
        return aResult;
    }

    public override ResD3 EvalD3(double U)
    {
        ResD3 aResult = new ResD3();
        Locate(U, out int aSpan, out double aNewU);
        Occt.BSplCLib.D3(aNewU, aSpan, myDeg, false, myPoles, myWeights, myKnots, myMults,
                    out aResult.Point, out aResult.D1, out aResult.D2, out aResult.D3);
        return aResult;
    }

    public override gp_Vec EvalDN(double U, int N)
    {
        Occt.BSplCLib.DN(U, N, 0, myDeg, false, myPoles, myWeights, myFlatKnots, Occt.BSplCLib.NoMults(), out gp_Vec V);
        return V;
    }

    protected internal override void Transform(in gp_Trsf T)
    {
        CheckMutable();
        for (int I = 1; I <= myPoles.Length(); I++)
        {
            myPoles[I].Transform(T);
        }
    }
}
#endif
