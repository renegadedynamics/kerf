// Port of: src/ModelingAlgorithms/TKOffset/BRepOffsetAPI/BRepOffsetAPI_MakePipeShell.hxx
// Port of: src/ModelingAlgorithms/TKOffset/BRepOffsetAPI/BRepOffsetAPI_MakePipeShell.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1998-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Filling;
using Kerf.Engine.Construction.Laws;
using Kerf.Engine.Construction.Surfaces;
using Kerf.Engine.Foundation;
using Kerf.Geometry;
using Kerf.Topology;

namespace Kerf.Engine.Construction.Api;

/// <summary>
/// Builds a shell (or, with <see cref="MakeSolid"/>, a solid) by sweeping one or more profiles along a wire (the
/// spine), under one of the trihedron modes (corrected Frenet by default) and a transition mode at the corners.
/// </summary>
/// <remarks>
/// <para>The profiles are placed on the spine as transformed copies (BRepFill_PipeShell), so neither they nor the
/// spine are written (decision 926). The result, <see cref="MakeSweep.FirstShape()"/>,
/// <see cref="MakeSweep.LastShape()"/>, the history and <see cref="Simulate"/>'s sections are frozen when
/// returned (design 4.2); <see cref="MakeShape.Generated"/> answers for the caller's own profiles,
/// their edges and vertices, and the spine's edges and vertices.</para>
/// </remarks>
internal class MakePipeShell : MakeSweep
{
    private readonly PipeShell _pipe;

    /// <summary>A pipe shell along <paramref name="spine"/>: add the profiles, choose the modes, then build.</summary>
    public MakePipeShell(in Wire spine)
    {
        _pipe = new PipeShell(spine);
        SetTolerance();
        SetTransitionMode();
        NotDone();
    }

    /// <summary>The Frenet trihedron (<paramref name="isFrenet"/>) or the corrected Frenet trihedron.</summary>
    public void SetMode(bool isFrenet = false)
    {
        _pipe.Set(isFrenet);
    }

    /// <summary>The discrete trihedron.</summary>
    public void SetDiscreteMode()
    {
        _pipe.SetDiscrete();
    }

    /// <summary>A fixed trihedron, given by <paramref name="axe"/>.</summary>
    public void SetMode(in Frame3D axe)
    {
        _pipe.Set(axe);
    }

    /// <summary>A trihedron with the constant binormal <paramref name="biNormal"/>.</summary>
    public void SetMode(in Direction3D biNormal)
    {
        _pipe.Set(biNormal);
    }

    /// <summary>The Darboux trihedron on <paramref name="spineSupport"/>; false if the spine isn't on it.</summary>
    public bool SetMode(in Shape spineSupport)
    {
        return _pipe.Set(spineSupport);
    }

    /// <summary>
    /// A trihedron guided by <paramref name="auxiliarySpine"/>, by the reduced curvilinear abscissa
    /// (<paramref name="curvilinearEquivalence"/>) or by the plane normal to the spine, with or without contact.
    /// </summary>
    public void SetMode(in Wire auxiliarySpine,
                        bool curvilinearEquivalence,
                        TypeOfContact keepContact = TypeOfContact.NoContact)
    {
        _pipe.Set(auxiliarySpine, curvilinearEquivalence, keepContact);
    }

    /// <summary>
    /// Adds the profile <paramref name="profile"/> (a wire or a vertex), translated onto the spine with
    /// <paramref name="withContact"/> and made orthogonal to it with <paramref name="withCorrection"/>.
    /// </summary>
    public void Add(in Shape profile, bool withContact = false, bool withCorrection = false)
    {
        _pipe.Add(profile, withContact, withCorrection);
    }

    /// <summary>Adds the profile <paramref name="profile"/> placed at the spine's vertex <paramref name="location"/>.</summary>
    public void Add(in Shape profile, in Vertex location, bool withContact = false, bool withCorrection = false)
    {
        _pipe.Add(profile, location, withContact, withCorrection);
    }

    /// <summary>The only profile <paramref name="profile"/>, scaled along the spine by the law <paramref name="l"/>.</summary>
    public void SetLaw(in Shape profile, LawsFunction l, bool withContact = false, bool withCorrection = false)
    {
        _pipe.SetLaw(profile, l, withContact, withCorrection);
    }

    /// <summary>
    /// The only profile <paramref name="profile"/>, placed at <paramref name="location"/> and scaled by the law
    /// <paramref name="l"/>.
    /// </summary>
    public void SetLaw(in Shape profile,
                       LawsFunction l,
                       in Vertex location,
                       bool withContact = false,
                       bool withCorrection = false)
    {
        _pipe.SetLaw(profile, l, location, withContact, withCorrection);
    }

    /// <summary>Removes the profile <paramref name="profile"/>.</summary>
    public void Delete(in Shape profile)
    {
        _pipe.DeleteProfile(profile);
    }

    /// <summary>True if a profile was added.</summary>
    public bool IsReady
    {
        get
        {
            return _pipe.IsReady;
        }
    }

    /// <summary>The status of the last build.</summary>
    public PipeError GetStatus()
    {
        SurfacePipeError stat;
        stat = _pipe.GetStatus();
        switch (stat)
        {
            case SurfacePipeError.PipeOk:
            {
                return PipeError.PipeDone;
            }
            case SurfacePipeError.PlaneNotIntersectGuide:
            {
                return PipeError.PlaneNotIntersectGuide;
            }
            case SurfacePipeError.ImpossibleContact:
            {
                return PipeError.ImpossibleContact;
            }
            default:
                return PipeError.PipeNotDone;
        }
    }

    /// <summary>The tolerances: 3D, of the bounds, and angular.</summary>
    public void SetTolerance(double tol3d = 1.0e-4, double boundTol = 1.0e-4, double tolAngular = 1.0e-2)
    {
        _pipe.SetTolerance(tol3d, boundTol, tolAngular);
    }

    /// <summary>The maximal degree of the swept surfaces (11 by default).</summary>
    public void SetMaxDegree(int newMaxDegree)
    {
        _pipe.SetMaxDegree(newMaxDegree);
    }

    /// <summary>The maximal number of segments of the swept surfaces (100 by default).</summary>
    public void SetMaxSegments(int newMaxSegments)
    {
        _pipe.SetMaxSegments(newMaxSegments);
    }

    //=======================================================================
    // function : SetForceApproxC1
    // purpose  : Set the flag that indicates attempt to approximate
    //           a C1-continuous surface if a swept surface proved
    //           to be C0.
    //=======================================================================
    /// <summary>Tries to approximate C1 surfaces where a swept surface is C0.</summary>
    public void SetForceApproxC1(bool forceApproxC1)
    {
        _pipe.SetForceApproxC1(forceApproxC1);
    }

    /// <summary>The treatment of the spine's corners: transformed (default), right corner or round corner.</summary>
    public void SetTransitionMode(global::Kerf.Construction.TransitionMode mode = global::Kerf.Construction.TransitionMode.Transformed)
    {
        _pipe.SetTransition((TransitionStyle)mode);
    }

    /// <summary>
    /// Fills <paramref name="r"/> with <paramref name="n"/> sections placed along the spine, without building the
    /// sweep (frozen).
    /// </summary>
    public void Simulate(int n, NCollection_List<Shape> r)
    {
        _pipe.Simulate(n, r);
    }

    /// <summary>Builds the shell.</summary>
    public override void Build(ProgressRange range = default)
    {
        bool ok;
        ok = _pipe.Build();
        if (ok)
        {
            _shape = _pipe.InternalShape();
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
        if (!IsDone)
        {
            throw new NotDoneException("BRepOffsetAPI_MakePipeShell::MakeSolid");
        }
        bool ok;
        ok = _pipe.MakeSolid();
        if (ok)
        {
            _shape = _pipe.InternalShape();
        }
        return ok;
    }

    internal override Shape InternalFirstShape() => _pipe.InternalFirstShape();

    internal override Shape InternalLastShape() => _pipe.InternalLastShape();

    /// <summary>The shapes generated from <paramref name="s"/>: a profile or a sub-shape of it, or of the spine.</summary>
    internal override NCollection_List<Shape> InternalGenerated(in Shape s)
    {
        _pipe.InternalGenerated(s, _generated);
        return _generated;
    }

    /// <summary>The approximation error of the swept surfaces.</summary>
    public double ErrorOnSurface()
    {
        return _pipe.ErrorOnSurface();
    }

    /// <summary>Whether <see cref="Build"/> builds the history (true by default).</summary>
    public void SetIsBuildHistory(bool isBuildHistory)
    {
        _pipe.SetIsBuildHistory(isBuildHistory);
    }

    /// <summary>True if <see cref="Build"/> builds the history.</summary>
    public bool IsBuildHistory => _pipe.IsBuildHistory;

    /// <summary>Appends the profiles, as given, to <paramref name="profiles"/>.</summary>
    public void Profiles(NCollection_List<Shape> profiles)
    {
        _pipe.Profiles(profiles);
    }

    /// <summary>The spine (frozen).</summary>
    public Wire Spine() => _pipe.Spine();
}
