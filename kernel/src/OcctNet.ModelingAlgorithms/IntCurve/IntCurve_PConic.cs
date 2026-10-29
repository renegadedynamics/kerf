// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntCurve/IntCurve_PConic.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntCurve/IntCurve_PConic.lxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntCurve/IntCurve_PConic.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GeomAbs_CurveType;

namespace Occt;

/// <summary>
/// A conic of gp seen as a parametric curve: an axis, two parameters and the curve type, with the tolerance and
/// the number of samples the intersection algorithms use on it.
/// </summary>
/// <remarks>
/// A struct, copied as OCCT copies it. A working tool (decision 54), so it has no equality;
/// <c>default(IntCurve_PConic)</c> is not a valid conic.
/// </remarks>
public struct IntCurve_PConic
{
    private gp_Ax22d axe;
    private double prm1;
    private double prm2;
    private double TheEpsX;
    private int TheAccuracy;
    private GeomAbs_CurveType type;

    public IntCurve_PConic(in gp_Elips2d E)
    {
        axe = E.Axis();
        prm1 = E.MajorRadius();
        prm2 = E.MinorRadius();
        TheEpsX = 0.00000001;
        TheAccuracy = 20;
        type = GeomAbs_Ellipse;
    }

    public IntCurve_PConic(in gp_Hypr2d H)
    {
        axe = H.Axis();
        prm1 = H.MajorRadius();
        prm2 = H.MinorRadius();
        TheEpsX = 0.00000001;
        TheAccuracy = 50;
        type = GeomAbs_Hyperbola;
    }

    public IntCurve_PConic(in gp_Circ2d C)
    {
        axe = C.Axis();
        prm1 = C.Radius();
        prm2 = 0.0;
        TheEpsX = 0.00000001;
        TheAccuracy = 20;
        type = GeomAbs_Circle;
    }

    public IntCurve_PConic(in gp_Parab2d P)
    {
        axe = P.Axis();
        prm1 = P.Focal();
        prm2 = 0.0;
        TheEpsX = 0.00000001;
        TheAccuracy = 20;
        type = GeomAbs_Parabola;
    }

    public IntCurve_PConic(in gp_Lin2d L)
    {
        axe = new gp_Ax22d(L.Position());
        prm1 = 0.0;
        prm2 = 0.0;
        TheEpsX = 0.00000001;
        TheAccuracy = 20;
        type = GeomAbs_Line;
    }

    /// <summary>The internal tolerance of the math algorithms, usually about 1e-10.</summary>
    internal void SetEpsX(double epsx)
    {
        TheEpsX = epsx;
    }

    /// <summary>The number of samples used to approximate the parametric curve on its domain.</summary>
    internal void SetAccuracy(int n)
    {
        TheAccuracy = n;
    }

    /// <summary>A copy with the internal tolerance <paramref name="epsx"/>.</summary>
    public readonly IntCurve_PConic WithEpsX(double epsx)
    {
        IntCurve_PConic aCopy = this;
        aCopy.SetEpsX(epsx);
        return aCopy;
    }

    /// <summary>A copy with <paramref name="n"/> samples.</summary>
    public readonly IntCurve_PConic WithAccuracy(int n)
    {
        IntCurve_PConic aCopy = this;
        aCopy.SetAccuracy(n);
        return aCopy;
    }

    public readonly int Accuracy() => (TheAccuracy);

    public readonly double EpsX() => (TheEpsX);

    /// <summary>The type of the conic.</summary>
    public readonly GeomAbs_CurveType TypeCurve() => (type);

    public readonly gp_Ax22d Axis2() => (axe);

    public readonly double Param1() => (prm1);

    public readonly double Param2() => (prm2);
}
