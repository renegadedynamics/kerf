// Port of: src/ModelingData/TKGeomBase/AppParCurves/AppParCurves_MultiBSpCurve.hxx
// Port of: src/ModelingData/TKGeomBase/AppParCurves/AppParCurves_MultiBSpCurve.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Approximation;

/// <summary>
/// A set of n B-spline curves with the same knots and degree approximating a MultiLine: the poles of a
/// <see cref="MultiCurve"/> with knots and multiplicities.
/// </summary>
/// <remarks>
/// <see cref="MultiBSpCurve(MultiBSpCurve)"/> is the C++ copy constructor: it shares the
/// poles, knots and multiplicities, and copies the degree. <see cref="Knots"/> and <see cref="Multiplicities"/>
/// return copies (decision 88), so a result can't be changed through them.
/// </remarks>
public class MultiBSpCurve : MultiCurve
{
    private NCollection_HArray1<double>? _myknots;
    private NCollection_HArray1<int>? _mymults;
    private int _degree;

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
    public MultiBSpCurve()
    {
        _degree = 0;
    }

    /// <summary>A MultiBSpCurve of <paramref name="nbPol"/> MultiPoints, to be set with <see cref="MultiCurve.SetValue"/>.</summary>
    public MultiBSpCurve(int nbPol) : base(nbPol)
    {
        _degree = 0;
    }

    /// <summary>A MultiBSpCurve of the poles <paramref name="tabMU"/>, knots and multiplicities (copied).</summary>
    public MultiBSpCurve(NCollection_Array1<MultiPoint> tabMU, NCollection_Array1<double> knots,
                                      NCollection_Array1<int> mults)
        : base(tabMU)
    {
        _myknots = new NCollection_HArray1<double>(knots);
        _mymults = new NCollection_HArray1<int>(mults);
        _degree = ComputeDegree(mults, NbPoles());
    }

    /// <summary>A MultiBSpCurve sharing the poles of <paramref name="sc"/>, with knots and multiplicities (copied).</summary>
    public MultiBSpCurve(MultiCurve sc, NCollection_Array1<double> knots, NCollection_Array1<int> mults)
        : base(sc)
    {
        _myknots = new NCollection_HArray1<double>(knots);
        _mymults = new NCollection_HArray1<int>(mults);
        _degree = ComputeDegree(mults, NbPoles());
    }

    /// <summary>OCCT's copy constructor: shares the poles, knots and multiplicities, copies the degree.</summary>
    public MultiBSpCurve(MultiBSpCurve other) : base(other)
    {
        _myknots = other._myknots;
        _mymults = other._mymults;
        _degree = other._degree;
    }

    /// <summary>Sets the knots (copied).</summary>
    public void SetKnots(NCollection_Array1<double> knots)
    {
        _myknots = new NCollection_HArray1<double>(knots);
    }

    /// <summary>Sets the multiplicities (copied), which also sets the degree.</summary>
    public void SetMultiplicities(NCollection_Array1<int> mults)
    {
        _mymults = new NCollection_HArray1<int>(mults);
        _degree = ComputeDegree(mults, NbPoles());
    }

    /// <summary>A copy of the knots.</summary>
    public NCollection_Array1<double> Knots() => new(_myknots!);

    /// <summary>A copy of the multiplicities.</summary>
    public NCollection_Array1<int> Multiplicities() => new(_mymults!);

    /// <summary>The degree of the curves.</summary>
    public override int Degree() => _degree;

    /// <summary>The point at <paramref name="u"/> of the 3d B-spline curve <paramref name="cuIndex"/>.</summary>
    public override void Value(int cuIndex, double u, out Point3D pt)
    {
        if (Dimension(cuIndex) != 3)
        {
            throw new OutOfRangeException();
        }
        NCollection_Array1<Point3D> tabPoles = new(1, _tabPoint!.Length());
        Curve(cuIndex, tabPoles);
        global::Kerf.Engine.Geometry.Splines.BSplCLib.D0(u, 0, _degree, false, tabPoles, global::Kerf.Engine.Geometry.Splines.BSplCLib.NoWeights(), _myknots!, _mymults, out pt);
    }

    /// <summary>The point at <paramref name="u"/> of the 2d B-spline curve <paramref name="cuIndex"/>.</summary>
    public override void Value(int cuIndex, double u, out Point2D pt)
    {
        if (Dimension(cuIndex) != 2)
        {
            throw new OutOfRangeException();
        }
        NCollection_Array1<Point2D> tabPoles = new(1, _tabPoint!.Length());
        Curve(cuIndex, tabPoles);
        global::Kerf.Engine.Geometry.Splines.BSplCLib.D0(u, 0, _degree, false, tabPoles, global::Kerf.Engine.Geometry.Splines.BSplCLib.NoWeights(), _myknots!, _mymults, out pt);
    }

    /// <summary>The point and first derivative at <paramref name="u"/> of the 3d B-spline curve <paramref name="cuIndex"/>.</summary>
    public override void D1(int cuIndex, double u, out Point3D pt, out Vector3D v1)
    {
        if (Dimension(cuIndex) != 3)
        {
            throw new OutOfRangeException();
        }
        NCollection_Array1<Point3D> tabPoles = new(1, _tabPoint!.Length());
        Curve(cuIndex, tabPoles);
        global::Kerf.Engine.Geometry.Splines.BSplCLib.D1(u, 0, _degree, false, tabPoles, global::Kerf.Engine.Geometry.Splines.BSplCLib.NoWeights(), _myknots!, _mymults, out pt, out v1);
    }

    /// <summary>The point and two derivatives at <paramref name="u"/> of the 3d B-spline curve <paramref name="cuIndex"/>.</summary>
    public override void D2(int cuIndex, double u, out Point3D pt, out Vector3D v1, out Vector3D v2)
    {
        if (Dimension(cuIndex) != 3)
        {
            throw new OutOfRangeException();
        }
        NCollection_Array1<Point3D> tabPoles = new(1, _tabPoint!.Length());
        Curve(cuIndex, tabPoles);
        global::Kerf.Engine.Geometry.Splines.BSplCLib.D2(u, 0, _degree, false, tabPoles, global::Kerf.Engine.Geometry.Splines.BSplCLib.NoWeights(), _myknots!, _mymults, out pt, out v1, out v2);
    }

    /// <summary>The point and first derivative at <paramref name="u"/> of the 2d B-spline curve <paramref name="cuIndex"/>.</summary>
    public override void D1(int cuIndex, double u, out Point2D pt, out Vector2D v1)
    {
        if (Dimension(cuIndex) != 2)
        {
            throw new OutOfRangeException();
        }
        NCollection_Array1<Point2D> tabPoles = new(1, _tabPoint!.Length());
        Curve(cuIndex, tabPoles);
        global::Kerf.Engine.Geometry.Splines.BSplCLib.D1(u, 0, _degree, false, tabPoles, global::Kerf.Engine.Geometry.Splines.BSplCLib.NoWeights(), _myknots!, _mymults, out pt, out v1);
    }

    /// <summary>The point and two derivatives at <paramref name="u"/> of the 2d B-spline curve <paramref name="cuIndex"/>.</summary>
    public override void D2(int cuIndex, double u, out Point2D pt, out Vector2D v1, out Vector2D v2)
    {
        if (Dimension(cuIndex) != 2)
        {
            throw new OutOfRangeException();
        }
        NCollection_Array1<Point2D> tabPoles = new(1, _tabPoint!.Length());
        Curve(cuIndex, tabPoles);
        global::Kerf.Engine.Geometry.Splines.BSplCLib.D2(u, 0, _degree, false, tabPoles, global::Kerf.Engine.Geometry.Splines.BSplCLib.NoWeights(), _myknots!, _mymults, out pt, out v1, out v2);
    }

    /// <summary>Writes the number of curves (OCCT's pole listing is commented out).</summary>
    public override void Dump(TextWriter o)
    {
        o.Write("AppParCurves_MultiBSpCurve dump:" + '\n');
        o.Write(" It contains " + MultiPoint.Int(NbCurves()) + " BSpline curves " + '\n');
        o.Write(" The poles are: " + '\n');
    }
}
