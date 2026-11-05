// Port of: src/ModelingData/TKBRep/BRep/BRep_TVertex.hxx
// Port of: src/ModelingData/TKBRep/BRep/BRep_TVertex.lxx
// Port of: src/ModelingData/TKBRep/BRep/BRep_TVertex.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Topology.Brep;

/// <summary>The geometry of a vertex: a 3D point, a tolerance and a list of point representations.</summary>
/// <remarks>
/// The setters and the list of representations are internal to OCCT.NET and throw
/// <see cref="ImmutableObjectException"/> once the vertex is frozen. <see cref="PointRepresentations"/> is the public,
/// read-only view of the list.
/// </remarks>
internal class BrepTVertex : TVertex
{
    private Point3D _pnt;
    private double _tolerance;
    private readonly NCollection_List<PointRepresentation> _points = new();

    /// <summary>An empty vertex at the origin, with a tolerance of RealEpsilon().</summary>
    public BrepTVertex()
    {
        _pnt = new Point3D();
        _tolerance = RealEpsilon();
    }

    /// <summary>The tolerance.</summary>
    public double Tolerance() => View()._tolerance;

    internal void Tolerance(double t)
    {
        if (WorkingOverlay.TryWritable(this, out TShape? working))
        {
            ((BrepTVertex)working).Tolerance(t);
            return;
        }

        CheckMutable();
        _tolerance = t;
    }

    /// <summary>Sets the tolerance to the max of <paramref name="t"/> and the current tolerance.</summary>
    internal void UpdateTolerance(double t)
    {
        if (t <= Tolerance() && WorkingOverlay.TakesWrite(this))
        {
            return; // Kerf: nothing to raise (unit 10H3)
        }
        if (WorkingOverlay.TryWritable(this, out TShape? working))
        {
            ((BrepTVertex)working).UpdateTolerance(t);
            return;
        }

        CheckMutable();
        if (t > _tolerance)
        {
            _tolerance = t;
        }
    }

    /// <summary>The point, in the vertex's own coordinate system.</summary>
    public Point3D Pnt() => View()._pnt;

    internal void Pnt(in Point3D p)
    {
        if (WorkingOverlay.TakesWrite(this) && View()._pnt.Equals(p))
        {
            return; // Kerf: the same point (unit 10H3)
        }
        if (WorkingOverlay.TryWritable(this, out TShape? working))
        {
            ((BrepTVertex)working).Pnt(p);
            return;
        }

        CheckMutable();
        _pnt = p;
    }

    /// <summary>The point representations (internal: ported code reads the list itself).</summary>
    internal NCollection_List<PointRepresentation> Points() => View()._points;

    internal NCollection_List<PointRepresentation> ChangePoints()
    {
        if (WorkingOverlay.TryWritable(this, out TShape? working))
        {
            return ((BrepTVertex)working).ChangePoints();
        }

        CheckMutable();
        return _points;
    }

    /// <summary>The point representations, as a new read-only list (OCCT.NET addition).</summary>
    public IReadOnlyList<PointRepresentation> PointRepresentations() => ImmutableArrays.Own([.. View()._points]);

    /// <summary>A new vertex with the same point and tolerance, without representations.</summary>
    public override TShape EmptyCopy()
    {
        BrepTVertex view = View();
        BrepTVertex tv = new BrepTVertex();
        tv.Pnt(view._pnt);
        tv.Tolerance(view._tolerance);
        return tv;
    }

    // The vertex whose data the current working overlay reads (WorkingOverlay): its working copy, or itself.
    private BrepTVertex View() => WorkingOverlay.TryView(this, out TShape? view) ? (BrepTVertex)view : this;

    internal override void FreezeRepresentations()
    {
        foreach (PointRepresentation pr in _points)
        {
            pr.Freeze();
        }
    }
}
