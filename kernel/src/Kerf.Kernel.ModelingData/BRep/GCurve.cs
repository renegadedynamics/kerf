// Port of: src/ModelingData/TKBRep/BRep/BRep_GCurve.hxx
// Port of: src/ModelingData/TKBRep/BRep/BRep_GCurve.lxx
// Port of: src/ModelingData/TKBRep/BRep/BRep_GCurve.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

using Kerf.Topology;
using Occt;

namespace Kerf.Engine.Topology.Brep;

/// <summary>The root of the geometric curve representations: a range, a first and a last parameter.</summary>
internal abstract class GCurve : CurveRepresentation
{
    private double _first;
    private double _last;

    protected GCurve(in Placement l, double first, double last) : base(l)
    {
        _first = first;
        _last = last;
    }

    internal void SetRange(double first, double last)
    {
        CheckMutable();
        _first = first;
        _last = last;
        Update();
    }

    /// <summary>The range of the curve.</summary>
    public void Range(out double first, out double last)
    {
        first = _first;
        last = _last;
    }

    /// <summary>The first parameter.</summary>
    public double First() => _first;

    /// <summary>The last parameter.</summary>
    public double Last() => _last;

    internal void First(double f)
    {
        CheckMutable();
        _first = f;
        Update();
    }

    internal void Last(double l)
    {
        CheckMutable();
        _last = l;
        Update();
    }

    /// <summary>The point at parameter <paramref name="u"/>.</summary>
    public abstract void D0(double u, out Point3D p);

    /// <summary>Recomputes any derived data after a modification; called when the range changes.</summary>
    internal virtual void Update()
    {
    }
}
