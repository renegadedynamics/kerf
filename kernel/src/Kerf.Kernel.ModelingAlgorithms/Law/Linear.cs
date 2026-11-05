// Port of: src/ModelingAlgorithms/TKGeomAlgo/Law/Law_Linear.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Law/Law_Linear.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using Kerf.Engine;

namespace Kerf.Engine.Construction.Laws;

/// <summary>A linear evolution law.</summary>
/// <remarks>A working object (decision 21): <see cref="Set"/> is public and changes the law.</remarks>
internal class Linear : LawsFunction
{
    private double _valdeb;
    private double _valfin;
    private double _pdeb;
    private double _pfin;

    /// <summary>An empty linear law.</summary>
    public Linear()
    {
        _valdeb = 0.0;
        _valfin = 0.0;
        _pdeb = 0.0;
        _pfin = 0.0;
    }

    /// <summary>Defines the law by the parametric bounds Pdeb and Pfin and the values Valdeb and Valfin there.</summary>
    public void Set(double pdeb, double valdeb, double pfin, double valfin)
    {
        _pdeb = pdeb;
        _pfin = pfin;
        _valdeb = valdeb;
        _valfin = valfin;
    }

    /// <summary>GeomAbs_CN.</summary>
    public override Continuity Continuity() => global::Kerf.Geometry.Continuity.CN;

    /// <summary>1.</summary>
    public override int NbIntervals(Continuity s) => 1;

    public override void Intervals(NCollection_Array1<double> t, Continuity s)
    {
        t.SetValue(t.Lower(), _pdeb);
        t.SetValue(t.Upper(), _pfin);
    }

    public override double Value(double x) => ((x - _pdeb) * _valfin + (_pfin - x) * _valdeb) / (_pfin - _pdeb);

    public override void D1(double x, out double f, out double d)
    {
        f = ((x - _pdeb) * _valfin + (_pfin - x) * _valdeb) / (_pfin - _pdeb);
        d = (_valfin - _valdeb) / (_pfin - _pdeb);
    }

    public override void D2(double x, out double f, out double d, out double d2)
    {
        f = ((x - _pdeb) * _valfin + (_pfin - x) * _valdeb) / (_pfin - _pdeb);
        d = (_valfin - _valdeb) / (_pfin - _pdeb);
        d2 = 0;
    }

    public override LawsFunction Trim(double pFirst, double pLast, double tol)
    {
        Linear l = new Linear();
        double vdeb, vfin;
        vdeb = ((pFirst - _pdeb) * _valfin + (_pfin - pFirst) * _valdeb) / (_pfin - _pdeb);
        vfin = ((pLast - _pdeb) * _valfin + (_pfin - pLast) * _valdeb) / (_pfin - _pdeb);
        l.Set(pFirst, vdeb, pLast, vfin);
        return l;
    }

    public override void Bounds(out double pFirst, out double pLast)
    {
        pFirst = _pdeb;
        pLast = _pfin;
    }
}
