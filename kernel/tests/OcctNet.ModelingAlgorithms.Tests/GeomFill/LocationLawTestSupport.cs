// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>A test trihedron law: the same orthonormal frame everywhere (T, N, T ^ N), with zero derivatives.</summary>
internal sealed class TestFixedTrihedron : GeomFill_TrihedronLaw
{
    private readonly gp_Vec myT;
    private readonly gp_Vec myN;

    public TestFixedTrihedron(gp_Vec theT, gp_Vec theN)
    {
        myT = theT.Normalized();
        myN = theN.Normalized();
    }

    public override GeomFill_TrihedronLaw Copy()
    {
        var aCopy = new TestFixedTrihedron(myT, myN);
        if (myCurve is not null)
        {
            aCopy.SetCurve(myCurve);
        }
        return aCopy;
    }

    public override bool D0(double Param, ref gp_Vec Tangent, ref gp_Vec Normal, ref gp_Vec BiNormal)
    {
        Tangent = myT;
        Normal = myN;
        BiNormal = myT.Crossed(myN);
        return true;
    }

    public override bool D1(double Param, ref gp_Vec Tangent, ref gp_Vec DTangent, ref gp_Vec Normal, ref gp_Vec DNormal,
                            ref gp_Vec BiNormal, ref gp_Vec DBiNormal)
    {
        D0(Param, ref Tangent, ref Normal, ref BiNormal);
        DTangent = DNormal = DBiNormal = new gp_Vec(0, 0, 0);
        return true;
    }

    public override bool D2(double Param, ref gp_Vec Tangent, ref gp_Vec DTangent, ref gp_Vec D2Tangent, ref gp_Vec Normal,
                            ref gp_Vec DNormal, ref gp_Vec D2Normal, ref gp_Vec BiNormal, ref gp_Vec DBiNormal,
                            ref gp_Vec D2BiNormal)
    {
        D0(Param, ref Tangent, ref Normal, ref BiNormal);
        DTangent = DNormal = DBiNormal = D2Tangent = D2Normal = D2BiNormal = new gp_Vec(0, 0, 0);
        return true;
    }

    public override int NbIntervals(GeomAbs_Shape S) => 1;

    public override void Intervals(NCollection_Array1<double> T, GeomAbs_Shape S)
    {
        T[T.Lower()] = myCurve!.FirstParameter();
        T[T.Upper()] = myCurve.LastParameter();
    }

    public override void GetAverageLaw(ref gp_Vec ATangent, ref gp_Vec ANormal, ref gp_Vec ABiNormal)
    {
        D0(0, ref ATangent, ref ANormal, ref ABiNormal);
    }

    public override bool IsConstant() => true;
}

/// <summary>
/// A test trihedron law with a guide: a fixed frame (T, N, T ^ N), the current point on the guide being the
/// guide's point at the same parameter.
/// </summary>
internal sealed class TestFixedGuideTrihedron : GeomFill_TrihedronWithGuide
{
    private readonly gp_Vec myT;
    private readonly gp_Vec myN;

    public TestFixedGuideTrihedron(Adaptor3d_Curve theGuide, gp_Vec theT, gp_Vec theN)
    {
        myGuide = theGuide;
        myTrimG = theGuide;
        myT = theT.Normalized();
        myN = theN.Normalized();
    }

    public int NbOrigine { get; private set; }

    public override Adaptor3d_Curve Guide() => myGuide!;

    public override void Origine(double Param1, double Param2) => NbOrigine++;

    public override GeomFill_TrihedronLaw Copy()
    {
        var aCopy = new TestFixedGuideTrihedron(myGuide!, myT, myN);
        if (myCurve is not null)
        {
            aCopy.SetCurve(myCurve);
        }
        return aCopy;
    }

    public override bool D0(double Param, ref gp_Vec Tangent, ref gp_Vec Normal, ref gp_Vec BiNormal)
    {
        Tangent = myT;
        Normal = myN;
        BiNormal = myT.Crossed(myN);
        myCurPointOnGuide = myGuide!.Value(Param);
        return true;
    }

    public override bool D1(double Param, ref gp_Vec Tangent, ref gp_Vec DTangent, ref gp_Vec Normal, ref gp_Vec DNormal,
                            ref gp_Vec BiNormal, ref gp_Vec DBiNormal)
    {
        D0(Param, ref Tangent, ref Normal, ref BiNormal);
        DTangent = DNormal = DBiNormal = new gp_Vec(0, 0, 0);
        return true;
    }

    public override bool D2(double Param, ref gp_Vec Tangent, ref gp_Vec DTangent, ref gp_Vec D2Tangent, ref gp_Vec Normal,
                            ref gp_Vec DNormal, ref gp_Vec D2Normal, ref gp_Vec BiNormal, ref gp_Vec DBiNormal,
                            ref gp_Vec D2BiNormal)
    {
        D0(Param, ref Tangent, ref Normal, ref BiNormal);
        DTangent = DNormal = DBiNormal = D2Tangent = D2Normal = D2BiNormal = new gp_Vec(0, 0, 0);
        return true;
    }

    public override int NbIntervals(GeomAbs_Shape S) => 1;

    public override void Intervals(NCollection_Array1<double> T, GeomAbs_Shape S)
    {
        T[T.Lower()] = myCurve!.FirstParameter();
        T[T.Upper()] = myCurve.LastParameter();
    }

    public override void GetAverageLaw(ref gp_Vec ATangent, ref gp_Vec ANormal, ref gp_Vec ABiNormal)
    {
        D0(0, ref ATangent, ref ANormal, ref ABiNormal);
    }
}

internal static class LocationLawTestSupport
{
    /// <summary>A B-spline curve through a few points, C2 inside (degree 3, one interior knot of multiplicity 1).</summary>
    internal static Geom_BSplineCurve WavyCurve()
    {
        var aPoles = new NCollection_Array1<gp_Pnt>(1, 5);
        aPoles[1] = new gp_Pnt(0, 0, 0);
        aPoles[2] = new gp_Pnt(1, 1, 0.2);
        aPoles[3] = new gp_Pnt(2, -0.5, 0.5);
        aPoles[4] = new gp_Pnt(3, 0.3, 0.1);
        aPoles[5] = new gp_Pnt(4, 0, 0.4);
        var aKnots = new NCollection_Array1<double>(1, 3);
        aKnots[1] = 0;
        aKnots[2] = 0.5;
        aKnots[3] = 1;
        var aMults = new NCollection_Array1<int>(1, 3);
        aMults[1] = 4;
        aMults[2] = 1;
        aMults[3] = 4;
        return new Geom_BSplineCurve(aPoles, aKnots, aMults, 3);
    }

    internal static void AssertNear(in gp_Vec theExpected, in gp_Vec theActual, double theTol, string theWhat)
    {
        Assert.True(theExpected.Subtracted(theActual).Magnitude() <= theTol,
                    $"{theWhat}: expected {theExpected}, got {theActual}");
    }

    internal static void AssertNear(in gp_XYZ theExpected, in gp_XYZ theActual, double theTol, string theWhat)
    {
        Assert.True(theExpected.Subtracted(theActual).Modulus() <= theTol,
                    $"{theWhat}: expected {theExpected}, got {theActual}");
    }

    internal static void AssertNear(in gp_Mat theExpected, in gp_Mat theActual, double theTol, string theWhat)
    {
        for (int i = 1; i <= 3; i++)
        {
            AssertNear(theExpected.Column(i), theActual.Column(i), theTol, $"{theWhat}, column {i}");
        }
    }

    internal static void AssertEqual(in gp_Mat theExpected, in gp_Mat theActual, string theWhat)
    {
        Assert.True(theExpected == theActual, $"{theWhat}: expected {theExpected}, got {theActual}");
    }
}
