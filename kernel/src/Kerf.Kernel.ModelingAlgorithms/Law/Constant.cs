// Port of: src/ModelingAlgorithms/TKGeomAlgo/Law/Law_Constant.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Law/Law_Constant.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Construction.Laws;

/// <summary>A constant law.</summary>
/// <remarks>A working object (decision 21): <see cref="Set"/> is public and changes the law.</remarks>
internal class Constant : LawsFunction
{
    private double _radius;
    private double _first;
    private double _last;

    public Constant()
    {
        _radius = 0.0;
        _first = 0.0;
        _last = 0.0;
    }

    /// <summary>Sets the value and the range of the law.</summary>
    public void Set(double radius, double pFirst, double pLast)
    {
        _radius = radius;
        _first = pFirst;
        _last = pLast;
    }

    /// <summary>GeomAbs_CN.</summary>
    public override Continuity Continuity() => global::Kerf.Geometry.Continuity.CN;

    /// <summary>1.</summary>
    public override int NbIntervals(Continuity s) => 1;

    public override void Intervals(NCollection_Array1<double> t, Continuity s)
    {
        t.SetValue(t.Lower(), _first);
        t.SetValue(t.Upper(), _last);
    }

    public override double Value(double x) => _radius;

    public override void D1(double x, out double f, out double d)
    {
        f = _radius;
        d = 0.0;
    }

    public override void D2(double x, out double f, out double d, out double d2)
    {
        f = _radius;
        d = d2 = 0.0;
    }

    public override LawsFunction Trim(double pFirst, double pLast, double tol)
    {
        Constant l = new Constant();
        l.Set(_radius, pFirst, pLast);
        return l;
    }

    public override void Bounds(out double pFirst, out double pLast)
    {
        pFirst = _first;
        pLast = _last;
    }
}
