// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Geometry;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>A test trihedron law: the same orthonormal frame everywhere (T, N, T ^ N), with zero derivatives.</summary>
internal sealed class TestFixedTrihedron : GeomFill_TrihedronLaw
{
    private readonly Vector3D myT;
    private readonly Vector3D myN;

    public TestFixedTrihedron(Vector3D theT, Vector3D theN)
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

    public override bool D0(double Param, ref Vector3D Tangent, ref Vector3D Normal, ref Vector3D BiNormal)
    {
        Tangent = myT;
        Normal = myN;
        BiNormal = myT.Crossed(myN);
        return true;
    }

    public override bool D1(double Param, ref Vector3D Tangent, ref Vector3D DTangent, ref Vector3D Normal, ref Vector3D DNormal,
                            ref Vector3D BiNormal, ref Vector3D DBiNormal)
    {
        D0(Param, ref Tangent, ref Normal, ref BiNormal);
        DTangent = DNormal = DBiNormal = new Vector3D(0, 0, 0);
        return true;
    }

    public override bool D2(double Param, ref Vector3D Tangent, ref Vector3D DTangent, ref Vector3D D2Tangent, ref Vector3D Normal,
                            ref Vector3D DNormal, ref Vector3D D2Normal, ref Vector3D BiNormal, ref Vector3D DBiNormal,
                            ref Vector3D D2BiNormal)
    {
        D0(Param, ref Tangent, ref Normal, ref BiNormal);
        DTangent = DNormal = DBiNormal = D2Tangent = D2Normal = D2BiNormal = new Vector3D(0, 0, 0);
        return true;
    }

    public override int NbIntervals(Continuity S) => 1;

    public override void Intervals(NCollection_Array1<double> T, Continuity S)
    {
        T[T.Lower()] = myCurve!.FirstParameter();
        T[T.Upper()] = myCurve.LastParameter();
    }

    public override void GetAverageLaw(ref Vector3D ATangent, ref Vector3D ANormal, ref Vector3D ABiNormal)
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
    private readonly Vector3D myT;
    private readonly Vector3D myN;

    public TestFixedGuideTrihedron(CurveAdaptorBase theGuide, Vector3D theT, Vector3D theN)
    {
        myGuide = theGuide;
        myTrimG = theGuide;
        myT = theT.Normalized();
        myN = theN.Normalized();
    }

    public int NbOrigine { get; private set; }

    public override CurveAdaptorBase Guide() => myGuide!;

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

    public override bool D0(double Param, ref Vector3D Tangent, ref Vector3D Normal, ref Vector3D BiNormal)
    {
        Tangent = myT;
        Normal = myN;
        BiNormal = myT.Crossed(myN);
        myCurPointOnGuide = myGuide!.Value(Param);
        return true;
    }

    public override bool D1(double Param, ref Vector3D Tangent, ref Vector3D DTangent, ref Vector3D Normal, ref Vector3D DNormal,
                            ref Vector3D BiNormal, ref Vector3D DBiNormal)
    {
        D0(Param, ref Tangent, ref Normal, ref BiNormal);
        DTangent = DNormal = DBiNormal = new Vector3D(0, 0, 0);
        return true;
    }

    public override bool D2(double Param, ref Vector3D Tangent, ref Vector3D DTangent, ref Vector3D D2Tangent, ref Vector3D Normal,
                            ref Vector3D DNormal, ref Vector3D D2Normal, ref Vector3D BiNormal, ref Vector3D DBiNormal,
                            ref Vector3D D2BiNormal)
    {
        D0(Param, ref Tangent, ref Normal, ref BiNormal);
        DTangent = DNormal = DBiNormal = D2Tangent = D2Normal = D2BiNormal = new Vector3D(0, 0, 0);
        return true;
    }

    public override int NbIntervals(Continuity S) => 1;

    public override void Intervals(NCollection_Array1<double> T, Continuity S)
    {
        T[T.Lower()] = myCurve!.FirstParameter();
        T[T.Upper()] = myCurve.LastParameter();
    }

    public override void GetAverageLaw(ref Vector3D ATangent, ref Vector3D ANormal, ref Vector3D ABiNormal)
    {
        D0(0, ref ATangent, ref ANormal, ref ABiNormal);
    }
}

internal static class LocationLawTestSupport
{
    /// <summary>A B-spline curve through a few points, C2 inside (degree 3, one interior knot of multiplicity 1).</summary>
    internal static BSplineCurve3D WavyCurve()
    {
        var aPoles = new NCollection_Array1<Point3D>(1, 5);
        aPoles[1] = new Point3D(0, 0, 0);
        aPoles[2] = new Point3D(1, 1, 0.2);
        aPoles[3] = new Point3D(2, -0.5, 0.5);
        aPoles[4] = new Point3D(3, 0.3, 0.1);
        aPoles[5] = new Point3D(4, 0, 0.4);
        var aKnots = new NCollection_Array1<double>(1, 3);
        aKnots[1] = 0;
        aKnots[2] = 0.5;
        aKnots[3] = 1;
        var aMults = new NCollection_Array1<int>(1, 3);
        aMults[1] = 4;
        aMults[2] = 1;
        aMults[3] = 4;
        return new BSplineCurve3D(aPoles, aKnots, aMults, 3);
    }

    internal static void AssertNear(in Vector3D theExpected, in Vector3D theActual, double theTol, string theWhat)
    {
        Assert.True(theExpected.Subtracted(theActual).Magnitude() <= theTol,
                    $"{theWhat}: expected {theExpected}, got {theActual}");
    }

    internal static void AssertNear(in Xyz theExpected, in Xyz theActual, double theTol, string theWhat)
    {
        Assert.True(theExpected.Subtracted(theActual).Modulus() <= theTol,
                    $"{theWhat}: expected {theExpected}, got {theActual}");
    }

    internal static void AssertNear(in Matrix3x3 theExpected, in Matrix3x3 theActual, double theTol, string theWhat)
    {
        for (int i = 1; i <= 3; i++)
        {
            AssertNear(theExpected.Column(i), theActual.Column(i), theTol, $"{theWhat}, column {i}");
        }
    }

    internal static void AssertEqual(in Matrix3x3 theExpected, in Matrix3x3 theActual, string theWhat)
    {
        Assert.True(theExpected == theActual, $"{theWhat}: expected {theExpected}, got {theActual}");
    }
}
