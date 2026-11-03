// Port of: src/ModelingData/TKGeomBase/AppParCurves/AppParCurves_MultiBSpCurve.hxx
// Port of: src/ModelingData/TKGeomBase/AppParCurves/AppParCurves_MultiBSpCurve.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Geometry;

namespace Occt;

/// <summary>
/// A set of n B-spline curves with the same knots and degree approximating a MultiLine: the poles of a
/// <see cref="AppParCurves_MultiCurve"/> with knots and multiplicities.
/// </summary>
/// <remarks>
/// <see cref="AppParCurves_MultiBSpCurve(AppParCurves_MultiBSpCurve)"/> is the C++ copy constructor: it shares the
/// poles, knots and multiplicities, and copies the degree. <see cref="Knots"/> and <see cref="Multiplicities"/>
/// return copies (decision 88), so a result can't be changed through them.
/// </remarks>
public class AppParCurves_MultiBSpCurve : AppParCurves_MultiCurve
{
    private NCollection_HArray1<double>? myknots;
    private NCollection_HArray1<int>? mymults;
    private int myDegree;

    private static int ComputeDegree(NCollection_Array1<int> mults, int nbPoles)
    {
        int i, sum = 0;
        for (i = mults.Lower(); i <= mults.Upper(); i++)
        {
            sum += mults[i];
        }
        return sum - nbPoles - 1;
    }

    /// <summary>An indefinite MultiBSpCurve.</summary>
    public AppParCurves_MultiBSpCurve()
    {
        myDegree = 0;
    }

    /// <summary>A MultiBSpCurve of <paramref name="NbPol"/> MultiPoints, to be set with <see cref="AppParCurves_MultiCurve.SetValue"/>.</summary>
    public AppParCurves_MultiBSpCurve(int NbPol) : base(NbPol)
    {
        myDegree = 0;
    }

    /// <summary>A MultiBSpCurve of the poles <paramref name="tabMU"/>, knots and multiplicities (copied).</summary>
    public AppParCurves_MultiBSpCurve(NCollection_Array1<AppParCurves_MultiPoint> tabMU, NCollection_Array1<double> Knots,
                                      NCollection_Array1<int> Mults)
        : base(tabMU)
    {
        myknots = new NCollection_HArray1<double>(Knots);
        mymults = new NCollection_HArray1<int>(Mults);
        myDegree = ComputeDegree(Mults, NbPoles());
    }

    /// <summary>A MultiBSpCurve sharing the poles of <paramref name="SC"/>, with knots and multiplicities (copied).</summary>
    public AppParCurves_MultiBSpCurve(AppParCurves_MultiCurve SC, NCollection_Array1<double> Knots, NCollection_Array1<int> Mults)
        : base(SC)
    {
        myknots = new NCollection_HArray1<double>(Knots);
        mymults = new NCollection_HArray1<int>(Mults);
        myDegree = ComputeDegree(Mults, NbPoles());
    }

    /// <summary>OCCT's copy constructor: shares the poles, knots and multiplicities, copies the degree.</summary>
    public AppParCurves_MultiBSpCurve(AppParCurves_MultiBSpCurve theOther) : base(theOther)
    {
        myknots = theOther.myknots;
        mymults = theOther.mymults;
        myDegree = theOther.myDegree;
    }

    /// <summary>Sets the knots (copied).</summary>
    public void SetKnots(NCollection_Array1<double> theKnots)
    {
        myknots = new NCollection_HArray1<double>(theKnots);
    }

    /// <summary>Sets the multiplicities (copied), which also sets the degree.</summary>
    public void SetMultiplicities(NCollection_Array1<int> theMults)
    {
        mymults = new NCollection_HArray1<int>(theMults);
        myDegree = ComputeDegree(theMults, NbPoles());
    }

    /// <summary>A copy of the knots.</summary>
    public NCollection_Array1<double> Knots() => new(myknots!);

    /// <summary>A copy of the multiplicities.</summary>
    public NCollection_Array1<int> Multiplicities() => new(mymults!);

    /// <summary>The degree of the curves.</summary>
    public override int Degree() => myDegree;

    /// <summary>The point at <paramref name="U"/> of the 3d B-spline curve <paramref name="CuIndex"/>.</summary>
    public override void Value(int CuIndex, double U, out Point3D Pt)
    {
        if (Dimension(CuIndex) != 3)
        {
            throw new OutOfRangeException();
        }
        NCollection_Array1<Point3D> TabPoles = new(1, tabPoint!.Length());
        Curve(CuIndex, TabPoles);
        global::Kerf.Engine.Geometry.Splines.BSplCLib.D0(U, 0, myDegree, false, TabPoles, global::Kerf.Engine.Geometry.Splines.BSplCLib.NoWeights(), myknots!, mymults, out Pt);
    }

    /// <summary>The point at <paramref name="U"/> of the 2d B-spline curve <paramref name="CuIndex"/>.</summary>
    public override void Value(int CuIndex, double U, out Point2D Pt)
    {
        if (Dimension(CuIndex) != 2)
        {
            throw new OutOfRangeException();
        }
        NCollection_Array1<Point2D> TabPoles = new(1, tabPoint!.Length());
        Curve(CuIndex, TabPoles);
        global::Kerf.Engine.Geometry.Splines.BSplCLib.D0(U, 0, myDegree, false, TabPoles, global::Kerf.Engine.Geometry.Splines.BSplCLib.NoWeights(), myknots!, mymults, out Pt);
    }

    /// <summary>The point and first derivative at <paramref name="U"/> of the 3d B-spline curve <paramref name="CuIndex"/>.</summary>
    public override void D1(int CuIndex, double U, out Point3D Pt, out Vector3D V1)
    {
        if (Dimension(CuIndex) != 3)
        {
            throw new OutOfRangeException();
        }
        NCollection_Array1<Point3D> TabPoles = new(1, tabPoint!.Length());
        Curve(CuIndex, TabPoles);
        global::Kerf.Engine.Geometry.Splines.BSplCLib.D1(U, 0, myDegree, false, TabPoles, global::Kerf.Engine.Geometry.Splines.BSplCLib.NoWeights(), myknots!, mymults, out Pt, out V1);
    }

    /// <summary>The point and two derivatives at <paramref name="U"/> of the 3d B-spline curve <paramref name="CuIndex"/>.</summary>
    public override void D2(int CuIndex, double U, out Point3D Pt, out Vector3D V1, out Vector3D V2)
    {
        if (Dimension(CuIndex) != 3)
        {
            throw new OutOfRangeException();
        }
        NCollection_Array1<Point3D> TabPoles = new(1, tabPoint!.Length());
        Curve(CuIndex, TabPoles);
        global::Kerf.Engine.Geometry.Splines.BSplCLib.D2(U, 0, myDegree, false, TabPoles, global::Kerf.Engine.Geometry.Splines.BSplCLib.NoWeights(), myknots!, mymults, out Pt, out V1, out V2);
    }

    /// <summary>The point and first derivative at <paramref name="U"/> of the 2d B-spline curve <paramref name="CuIndex"/>.</summary>
    public override void D1(int CuIndex, double U, out Point2D Pt, out Vector2D V1)
    {
        if (Dimension(CuIndex) != 2)
        {
            throw new OutOfRangeException();
        }
        NCollection_Array1<Point2D> TabPoles = new(1, tabPoint!.Length());
        Curve(CuIndex, TabPoles);
        global::Kerf.Engine.Geometry.Splines.BSplCLib.D1(U, 0, myDegree, false, TabPoles, global::Kerf.Engine.Geometry.Splines.BSplCLib.NoWeights(), myknots!, mymults, out Pt, out V1);
    }

    /// <summary>The point and two derivatives at <paramref name="U"/> of the 2d B-spline curve <paramref name="CuIndex"/>.</summary>
    public override void D2(int CuIndex, double U, out Point2D Pt, out Vector2D V1, out Vector2D V2)
    {
        if (Dimension(CuIndex) != 2)
        {
            throw new OutOfRangeException();
        }
        NCollection_Array1<Point2D> TabPoles = new(1, tabPoint!.Length());
        Curve(CuIndex, TabPoles);
        global::Kerf.Engine.Geometry.Splines.BSplCLib.D2(U, 0, myDegree, false, TabPoles, global::Kerf.Engine.Geometry.Splines.BSplCLib.NoWeights(), myknots!, mymults, out Pt, out V1, out V2);
    }

    /// <summary>Writes the number of curves (OCCT's pole listing is commented out).</summary>
    public override void Dump(TextWriter o)
    {
        o.Write("AppParCurves_MultiBSpCurve dump:" + '\n');
        o.Write(" It contains " + AppParCurves_MultiPoint.Int(NbCurves()) + " BSpline curves " + '\n');
        o.Write(" The poles are: " + '\n');
    }
}
