// Port of: src/ModelingData/TKBRep/BRep/BRep_TEdge.hxx
// Port of: src/ModelingData/TKBRep/BRep/BRep_TEdge.lxx
// Port of: src/ModelingData/TKBRep/BRep/BRep_TEdge.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Bento.Collections;

namespace Occt;

/// <summary>
/// The geometry of an edge: a tolerance, the SameParameter, SameRange and Degenerated flags, and a list of curve
/// representations.
/// </summary>
/// <remarks>
/// The setters and the list of representations are internal to OCCT.NET and throw
/// <see cref="Standard_ImmutableObject"/> once the edge is frozen. <see cref="CurveRepresentations"/> is the public,
/// read-only view of the list.
/// </remarks>
public class BRep_TEdge : TopoDS_TEdge
{
    private const int ParameterMask = 1;
    private const int RangeMask = 2;
    private const int DegeneratedMask = 4;

    private double myTolerance;
    private int myFlags;
    private readonly NCollection_List<BRep_CurveRepresentation> myCurves = new();

    /// <summary>An empty edge: tolerance RealEpsilon(), SameParameter and SameRange.</summary>
    public BRep_TEdge()
    {
        myTolerance = RealEpsilon();
        myFlags = 0;
        SameParameter(true);
        SameRange(true);
    }

    /// <summary>The tolerance.</summary>
    public double Tolerance() => myTolerance;

    internal void Tolerance(double T)
    {
        CheckMutable();
        myTolerance = T;
    }

    /// <summary>Sets the tolerance to the max of <paramref name="T"/> and the current tolerance.</summary>
    internal void UpdateTolerance(double T)
    {
        CheckMutable();
        if (T > myTolerance)
        {
            myTolerance = T;
        }
    }

    /// <summary>The SameParameter flag.</summary>
    public bool SameParameter() => (myFlags & ParameterMask) != 0;

    internal void SameParameter(bool S)
    {
        CheckMutable();
        if (S)
        {
            myFlags |= ParameterMask;
        }
        else
        {
            myFlags &= ~ParameterMask;
        }
    }

    /// <summary>The SameRange flag.</summary>
    public bool SameRange() => (myFlags & RangeMask) != 0;

    internal void SameRange(bool S)
    {
        CheckMutable();
        if (S)
        {
            myFlags |= RangeMask;
        }
        else
        {
            myFlags &= ~RangeMask;
        }
    }

    /// <summary>The Degenerated flag.</summary>
    public bool Degenerated() => (myFlags & DegeneratedMask) != 0;

    internal void Degenerated(bool S)
    {
        CheckMutable();
        if (S)
        {
            myFlags |= DegeneratedMask;
        }
        else
        {
            myFlags &= ~DegeneratedMask;
        }
    }

    /// <summary>The curve representations (internal: ported code reads the list itself).</summary>
    internal NCollection_List<BRep_CurveRepresentation> Curves() => myCurves;

    internal NCollection_List<BRep_CurveRepresentation> ChangeCurves()
    {
        CheckMutable();
        return myCurves;
    }

    /// <summary>The curve representations, as a new read-only list (OCCT.NET addition).</summary>
    public IBentoList<BRep_CurveRepresentation> CurveRepresentations() => PackedList.Own([.. myCurves]);

    /// <summary>
    /// A new edge with the same tolerance, flags and curve representations (copied, sharing the geometry), without
    /// polygons and sub-shapes.
    /// </summary>
    public override TopoDS_TShape EmptyCopy()
    {
        BRep_TEdge TE = new BRep_TEdge();
        TE.Tolerance(myTolerance);
        // copy the curves representations
        NCollection_List<BRep_CurveRepresentation> l = TE.ChangeCurves();
        var itr = new NCollection_List<BRep_CurveRepresentation>.Iterator(myCurves);

        while (itr.More())
        {
            // do NOT copy polygons
            if (itr.Value() is BRep_GCurve || itr.Value() is BRep_CurveOn2Surfaces)
            {
                l.Append(itr.Value().Copy());
            }

            itr.Next();
        }

        TE.Degenerated(Degenerated());
        TE.SameParameter(SameParameter());
        TE.SameRange(SameRange());

        return TE;
    }

    internal override void FreezeRepresentations()
    {
        foreach (BRep_CurveRepresentation aCR in myCurves)
        {
            aCR.Freeze();
        }
    }
}
