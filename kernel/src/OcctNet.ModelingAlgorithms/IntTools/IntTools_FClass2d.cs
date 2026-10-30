// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

// TEMPORARY API stub of IntTools_FClass2d (unit 6B); replaced by the port in this unit before merging.
/// <summary>The 2D point classifier of a face.</summary>
public sealed class IntTools_FClass2d
{
    /// <summary>Empty classifier.</summary>
    public IntTools_FClass2d() { }

    /// <summary>The classifier of the face with the tolerance.</summary>
    public IntTools_FClass2d(in TopoDS_Face F, double Tol) => Init(F, Tol);

    /// <summary>Initializes the classifier with the face and tolerance.</summary>
    public void Init(in TopoDS_Face F, double Tol) => throw new NotImplementedException();

    /// <summary>The state of the infinite 2D point.</summary>
    public TopAbs_State PerformInfinitePoint() => throw new NotImplementedException();

    /// <summary>The state of the 2D point.</summary>
    public TopAbs_State Perform(in gp_Pnt2d Puv, bool RecadreOnPeriodic = true) => throw new NotImplementedException();

    /// <summary>Tests the point with offsets.</summary>
    public TopAbs_State TestOnRestriction(in gp_Pnt2d Puv, double Tol, bool RecadreOnPeriodic = true) => throw new NotImplementedException();

    /// <summary>True if the face is a hole.</summary>
    public bool IsHole() => throw new NotImplementedException();
}
