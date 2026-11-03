// Port of: src/ModelingAlgorithms/TKOffset/BRepOffsetAPI/BRepOffsetAPI_MakePipeShell.hxx
// Port of: src/ModelingAlgorithms/TKOffset/BRepOffsetAPI/BRepOffsetAPI_MakePipeShell.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1998-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Geometry;
using Kerf.Topology;
using static Occt.BRepBuilderAPI_PipeError;
using static Occt.BRepBuilderAPI_TransitionMode;
using static Occt.BRepFill_TypeOfContact;
using static Occt.GeomFill_PipeError;

namespace Occt;

/// <summary>
/// Builds a shell (or, with <see cref="MakeSolid"/>, a solid) by sweeping one or more profiles along a wire (the
/// spine), under one of the trihedron modes (corrected Frenet by default) and a transition mode at the corners.
/// </summary>
/// <remarks>
/// <para>The profiles are placed on the spine as transformed copies (BRepFill_PipeShell), so neither they nor the
/// spine are written (decision 926). The result, <see cref="BRepPrimAPI_MakeSweep.FirstShape()"/>,
/// <see cref="BRepPrimAPI_MakeSweep.LastShape()"/>, the history and <see cref="Simulate"/>'s sections are frozen when
/// returned (design 4.2); <see cref="BRepBuilderAPI_MakeShape.Generated"/> answers for the caller's own profiles,
/// their edges and vertices, and the spine's edges and vertices.</para>
/// </remarks>
public class BRepOffsetAPI_MakePipeShell : BRepPrimAPI_MakeSweep
{
    private readonly BRepFill_PipeShell myPipe;

    /// <summary>A pipe shell along <paramref name="Spine"/>: add the profiles, choose the modes, then build.</summary>
    public BRepOffsetAPI_MakePipeShell(in Wire Spine)
    {
        myPipe = new BRepFill_PipeShell(Spine);
        SetTolerance();
        SetTransitionMode();
        NotDone();
    }

    /// <summary>The Frenet trihedron (<paramref name="IsFrenet"/>) or the corrected Frenet trihedron.</summary>
    public void SetMode(bool IsFrenet = false)
    {
        myPipe.Set(IsFrenet);
    }

    /// <summary>The discrete trihedron.</summary>
    public void SetDiscreteMode()
    {
        myPipe.SetDiscrete();
    }

    /// <summary>A fixed trihedron, given by <paramref name="Axe"/>.</summary>
    public void SetMode(in Frame3D Axe)
    {
        myPipe.Set(Axe);
    }

    /// <summary>A trihedron with the constant binormal <paramref name="BiNormal"/>.</summary>
    public void SetMode(in Direction3D BiNormal)
    {
        myPipe.Set(BiNormal);
    }

    /// <summary>The Darboux trihedron on <paramref name="SpineSupport"/>; false if the spine isn't on it.</summary>
    public bool SetMode(in Shape SpineSupport)
    {
        return myPipe.Set(SpineSupport);
    }

    /// <summary>
    /// A trihedron guided by <paramref name="AuxiliarySpine"/>, by the reduced curvilinear abscissa
    /// (<paramref name="CurvilinearEquivalence"/>) or by the plane normal to the spine, with or without contact.
    /// </summary>
    public void SetMode(in Wire AuxiliarySpine,
                        bool CurvilinearEquivalence,
                        BRepFill_TypeOfContact KeepContact = BRepFill_NoContact)
    {
        myPipe.Set(AuxiliarySpine, CurvilinearEquivalence, KeepContact);
    }

    /// <summary>
    /// Adds the profile <paramref name="Profile"/> (a wire or a vertex), translated onto the spine with
    /// <paramref name="WithContact"/> and made orthogonal to it with <paramref name="WithCorrection"/>.
    /// </summary>
    public void Add(in Shape Profile, bool WithContact = false, bool WithCorrection = false)
    {
        myPipe.Add(Profile, WithContact, WithCorrection);
    }

    /// <summary>Adds the profile <paramref name="Profile"/> placed at the spine's vertex <paramref name="Location"/>.</summary>
    public void Add(in Shape Profile, in Vertex Location, bool WithContact = false, bool WithCorrection = false)
    {
        myPipe.Add(Profile, Location, WithContact, WithCorrection);
    }

    /// <summary>The only profile <paramref name="Profile"/>, scaled along the spine by the law <paramref name="L"/>.</summary>
    public void SetLaw(in Shape Profile, Law_Function L, bool WithContact = false, bool WithCorrection = false)
    {
        myPipe.SetLaw(Profile, L, WithContact, WithCorrection);
    }

    /// <summary>
    /// The only profile <paramref name="Profile"/>, placed at <paramref name="Location"/> and scaled by the law
    /// <paramref name="L"/>.
    /// </summary>
    public void SetLaw(in Shape Profile,
                       Law_Function L,
                       in Vertex Location,
                       bool WithContact = false,
                       bool WithCorrection = false)
    {
        myPipe.SetLaw(Profile, L, Location, WithContact, WithCorrection);
    }

    /// <summary>Removes the profile <paramref name="Profile"/>.</summary>
    public void Delete(in Shape Profile)
    {
        myPipe.DeleteProfile(Profile);
    }

    /// <summary>True if a profile was added.</summary>
    public bool IsReady()
    {
        return myPipe.IsReady();
    }

    /// <summary>The status of the last build.</summary>
    public BRepBuilderAPI_PipeError GetStatus()
    {
        GeomFill_PipeError stat;
        stat = myPipe.GetStatus();
        switch (stat)
        {
            case GeomFill_PipeOk:
            {
                return BRepBuilderAPI_PipeDone;
            }
            case GeomFill_PlaneNotIntersectGuide:
            {
                return BRepBuilderAPI_PlaneNotIntersectGuide;
            }
            case GeomFill_ImpossibleContact:
            {
                return BRepBuilderAPI_ImpossibleContact;
            }
            default:
                return BRepBuilderAPI_PipeNotDone;
        }
    }

    /// <summary>The tolerances: 3D, of the bounds, and angular.</summary>
    public void SetTolerance(double Tol3d = 1.0e-4, double BoundTol = 1.0e-4, double TolAngular = 1.0e-2)
    {
        myPipe.SetTolerance(Tol3d, BoundTol, TolAngular);
    }

    /// <summary>The maximal degree of the swept surfaces (11 by default).</summary>
    public void SetMaxDegree(int NewMaxDegree)
    {
        myPipe.SetMaxDegree(NewMaxDegree);
    }

    /// <summary>The maximal number of segments of the swept surfaces (100 by default).</summary>
    public void SetMaxSegments(int NewMaxSegments)
    {
        myPipe.SetMaxSegments(NewMaxSegments);
    }

    //=======================================================================
    // function : SetForceApproxC1
    // purpose  : Set the flag that indicates attempt to approximate
    //           a C1-continuous surface if a swept surface proved
    //           to be C0.
    //=======================================================================
    /// <summary>Tries to approximate C1 surfaces where a swept surface is C0.</summary>
    public void SetForceApproxC1(bool ForceApproxC1)
    {
        myPipe.SetForceApproxC1(ForceApproxC1);
    }

    /// <summary>The treatment of the spine's corners: transformed (default), right corner or round corner.</summary>
    public void SetTransitionMode(BRepBuilderAPI_TransitionMode Mode = BRepBuilderAPI_Transformed)
    {
        myPipe.SetTransition((BRepFill_TransitionStyle)Mode);
    }

    /// <summary>
    /// Fills <paramref name="R"/> with <paramref name="N"/> sections placed along the spine, without building the
    /// sweep (frozen).
    /// </summary>
    public void Simulate(int N, NCollection_List<Shape> R)
    {
        myPipe.Simulate(N, R);
    }

    /// <summary>Builds the shell.</summary>
    public override void Build(ProgressRange theRange = default)
    {
        bool Ok;
        Ok = myPipe.Build();
        if (Ok)
        {
            myShape = myPipe.InternalShape();
            Done();
        }
        else
        {
            NotDone();
        }
    }

    /// <summary>Closes the shell with planar faces at its ends and makes a solid of it; false if it can't.</summary>
    /// <exception cref="NotDoneException">The shell isn't built.</exception>
    public bool MakeSolid()
    {
        if (!IsDone())
        {
            throw new NotDoneException("BRepOffsetAPI_MakePipeShell::MakeSolid");
        }
        bool Ok;
        Ok = myPipe.MakeSolid();
        if (Ok)
        {
            myShape = myPipe.InternalShape();
        }
        return Ok;
    }

    internal override Shape InternalFirstShape() => myPipe.InternalFirstShape();

    internal override Shape InternalLastShape() => myPipe.InternalLastShape();

    /// <summary>The shapes generated from <paramref name="S"/>: a profile or a sub-shape of it, or of the spine.</summary>
    internal override NCollection_List<Shape> InternalGenerated(in Shape S)
    {
        myPipe.InternalGenerated(S, myGenerated);
        return myGenerated;
    }

    /// <summary>The approximation error of the swept surfaces.</summary>
    public double ErrorOnSurface()
    {
        return myPipe.ErrorOnSurface();
    }

    /// <summary>Whether <see cref="Build"/> builds the history (true by default).</summary>
    public void SetIsBuildHistory(bool theIsBuildHistory)
    {
        myPipe.SetIsBuildHistory(theIsBuildHistory);
    }

    /// <summary>True if <see cref="Build"/> builds the history.</summary>
    public bool IsBuildHistory() => myPipe.IsBuildHistory();

    /// <summary>Appends the profiles, as given, to <paramref name="theProfiles"/>.</summary>
    public void Profiles(NCollection_List<Shape> theProfiles)
    {
        myPipe.Profiles(theProfiles);
    }

    /// <summary>The spine (frozen).</summary>
    public Wire Spine() => myPipe.Spine();
}
