// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntCurve/IntCurve_PConic.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntCurve/IntCurve_PConic.lxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntCurve/IntCurve_PConic.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using static Kerf.Geometry.CurveKind;

namespace Kerf.Engine.Intersection.Curves;

/// <summary>
/// A conic of gp seen as a parametric curve: an axis, two parameters and the curve type, with the tolerance and
/// the number of samples the intersection algorithms use on it.
/// </summary>
/// <remarks>
/// A struct, copied as OCCT copies it. A working tool (decision 54), so it has no equality;
/// <c>default(IntCurve_PConic)</c> is not a valid conic.
/// </remarks>
internal struct PConic
{
    private Frame2D _axe;
    private double _prm1;
    private double _prm2;
    private double _theEpsX;
    private int _theAccuracy;
    private CurveKind _type;

    public PConic(in Ellipse2D e)
    {
        _axe = e.Axis;
        _prm1 = e.MajorRadius;
        _prm2 = e.MinorRadius;
        _theEpsX = 0.00000001;
        _theAccuracy = 20;
        _type = Ellipse;
    }

    public PConic(in Hyperbola2D h)
    {
        _axe = h.Axis;
        _prm1 = h.MajorRadius;
        _prm2 = h.MinorRadius;
        _theEpsX = 0.00000001;
        _theAccuracy = 50;
        _type = Hyperbola;
    }

    public PConic(in Circle2D c)
    {
        _axe = c.Axis;
        _prm1 = c.Radius;
        _prm2 = 0.0;
        _theEpsX = 0.00000001;
        _theAccuracy = 20;
        _type = Circle;
    }

    public PConic(in Parabola2D p)
    {
        _axe = p.Axis;
        _prm1 = p.Focal;
        _prm2 = 0.0;
        _theEpsX = 0.00000001;
        _theAccuracy = 20;
        _type = Parabola;
    }

    public PConic(in Line2D l)
    {
        _axe = new Frame2D(l.Position);
        _prm1 = 0.0;
        _prm2 = 0.0;
        _theEpsX = 0.00000001;
        _theAccuracy = 20;
        _type = Line;
    }

    /// <summary>The internal tolerance of the math algorithms, usually about 1e-10.</summary>
    internal void SetEpsX(double epsx)
    {
        _theEpsX = epsx;
    }

    /// <summary>The number of samples used to approximate the parametric curve on its domain.</summary>
    internal void SetAccuracy(int n)
    {
        _theAccuracy = n;
    }

    /// <summary>A copy with the internal tolerance <paramref name="epsx"/>.</summary>
    public readonly PConic WithEpsX(double epsx)
    {
        PConic copy = this;
        copy.SetEpsX(epsx);
        return copy;
    }

    /// <summary>A copy with <paramref name="n"/> samples.</summary>
    public readonly PConic WithAccuracy(int n)
    {
        PConic copy = this;
        copy.SetAccuracy(n);
        return copy;
    }

    public readonly int Accuracy() => (_theAccuracy);

    public readonly double EpsX() => (_theEpsX);

    /// <summary>The type of the conic.</summary>
    public readonly CurveKind TypeCurve() => (_type);

    public readonly Frame2D Axis2() => (_axe);

    public readonly double Param1() => (_prm1);

    public readonly double Param2() => (_prm2);
}
