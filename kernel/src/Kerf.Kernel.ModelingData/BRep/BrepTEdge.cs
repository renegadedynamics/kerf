// Port of: src/ModelingData/TKBRep/BRep/BRep_TEdge.hxx
// Port of: src/ModelingData/TKBRep/BRep/BRep_TEdge.lxx
// Port of: src/ModelingData/TKBRep/BRep/BRep_TEdge.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Topology.Tools;
using Occt;

namespace Kerf.Engine.Topology.Brep;

/// <summary>
/// The geometry of an edge: a tolerance, the SameParameter, SameRange and Degenerated flags, and a list of curve
/// representations.
/// </summary>
/// <remarks>
/// The setters and the list of representations are internal to Kerf and throw
/// <see cref="ImmutableObjectException"/> once the edge is frozen, unless a working overlay is open: then they write
/// into its working copy of the edge, which the getters read while it is open (<see cref="WorkingOverlay"/>, Kerf unit
/// 10H1). <see cref="CurveRepresentations"/> is the public, read-only view of the list.
/// </remarks>
internal class BrepTEdge : TEdge
{
    private const int ParameterMask = 1;
    private const int RangeMask = 2;
    private const int DegeneratedMask = 4;

    private double _tolerance;
    private int _flags;
    private readonly NCollection_List<CurveRepresentation> _curves = new();

    /// <summary>An empty edge: tolerance RealEpsilon(), SameParameter and SameRange.</summary>
    public BrepTEdge()
    {
        _tolerance = RealEpsilon();
        _flags = 0;
        SameParameter(true);
        SameRange(true);
    }

    /// <summary>The tolerance.</summary>
    public double Tolerance() => View()._tolerance;

    internal void Tolerance(double t)
    {
        if (WorkingOverlay.TryWritable(this, out TShape? working))
        {
            ((BrepTEdge)working).Tolerance(t);
            return;
        }

        CheckMutable();
        _tolerance = t;
    }

    /// <summary>Sets the tolerance to the max of <paramref name="t"/> and the current tolerance.</summary>
    internal void UpdateTolerance(double t)
    {
        if (WorkingOverlay.TryWritable(this, out TShape? working))
        {
            ((BrepTEdge)working).UpdateTolerance(t);
            return;
        }

        CheckMutable();
        if (t > _tolerance)
        {
            _tolerance = t;
        }
    }

    /// <summary>The SameParameter flag.</summary>
    public bool SameParameter() => (View()._flags & ParameterMask) != 0;

    internal void SameParameter(bool s)
    {
        if (WorkingOverlay.TryWritable(this, out TShape? working))
        {
            ((BrepTEdge)working).SameParameter(s);
            return;
        }

        CheckMutable();
        if (s)
        {
            _flags |= ParameterMask;
        }
        else
        {
            _flags &= ~ParameterMask;
        }
    }

    /// <summary>The SameRange flag.</summary>
    public bool SameRange() => (View()._flags & RangeMask) != 0;

    internal void SameRange(bool s)
    {
        if (WorkingOverlay.TryWritable(this, out TShape? working))
        {
            ((BrepTEdge)working).SameRange(s);
            return;
        }

        CheckMutable();
        if (s)
        {
            _flags |= RangeMask;
        }
        else
        {
            _flags &= ~RangeMask;
        }
    }

    /// <summary>The Degenerated flag.</summary>
    public bool Degenerated() => (View()._flags & DegeneratedMask) != 0;

    internal void Degenerated(bool s)
    {
        if (WorkingOverlay.TryWritable(this, out TShape? working))
        {
            ((BrepTEdge)working).Degenerated(s);
            return;
        }

        CheckMutable();
        if (s)
        {
            _flags |= DegeneratedMask;
        }
        else
        {
            _flags &= ~DegeneratedMask;
        }
    }

    /// <summary>The curve representations (internal: ported code reads the list itself).</summary>
    internal NCollection_List<CurveRepresentation> Curves() => View()._curves;

    internal NCollection_List<CurveRepresentation> ChangeCurves()
    {
        if (WorkingOverlay.TryWritable(this, out TShape? working))
        {
            return ((BrepTEdge)working).ChangeCurves();
        }

        CheckMutable();
        return _curves;
    }

    /// <summary>The curve representations, as a new read-only list (Kerf addition).</summary>
    public IReadOnlyList<CurveRepresentation> CurveRepresentations() => ImmutableArrays.Own([.. View()._curves]);

    /// <summary>
    /// A new edge with the same tolerance, flags and curve representations (copied, sharing the geometry), without
    /// polygons and sub-shapes.
    /// </summary>
    public override TShape EmptyCopy()
    {
        BrepTEdge view = View();
        BrepTEdge te = new BrepTEdge();
        te.Tolerance(view._tolerance);
        // copy the curves representations
        NCollection_List<CurveRepresentation> l = te.ChangeCurves();
        var itr = new NCollection_List<CurveRepresentation>.Iterator(view._curves);

        while (itr.More())
        {
            // do NOT copy polygons
            if (itr.Value() is GCurve || itr.Value() is CurveOn2Surfaces)
            {
                l.Append(itr.Value().Copy());
            }

            itr.Next();
        }

        te.Degenerated(view.Degenerated());
        te.SameParameter(SameParameter());
        te.SameRange(SameRange());

        return te;
    }

    // The edge whose data the current working overlay reads (WorkingOverlay): its working copy, or itself.
    private BrepTEdge View() => WorkingOverlay.TryView(this, out TShape? view) ? (BrepTEdge)view : this;

    internal override void FreezeRepresentations()
    {
        foreach (CurveRepresentation cr in _curves)
        {
            cr.FreezeContents(); // the meshes the mesher attached unfrozen (decision 488)
            cr.Freeze();
        }
    }
}
