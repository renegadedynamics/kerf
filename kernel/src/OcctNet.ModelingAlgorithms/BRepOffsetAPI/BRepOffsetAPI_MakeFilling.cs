// Port of: src/ModelingAlgorithms/TKOffset/BRepOffsetAPI/BRepOffsetAPI_MakeFilling.hxx
// Port of: src/ModelingAlgorithms/TKOffset/BRepOffsetAPI/BRepOffsetAPI_MakeFilling.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// N-sided filling (BRepFill_Filling as a maker): a face bounded by edges, each possibly G0, G1 or G2 continuous with
/// a support face, through further curve and point constraints.
/// </summary>
/// <remarks>
/// A maker (decisions 414, 454): <c>Shape()</c> freezes the face, <c>Generated</c> lists frozen shapes (the new edge
/// or vertex made from a boundary edge or vertex). The caller's edges and faces are never written. As in OCCT,
/// <see cref="Build"/> may be called again after more constraints are added (build123d.NET's <c>MakeSurface</c>
/// adds points to a built filling): it builds a new face, and the face already handed out doesn't change.
/// </remarks>
public class BRepOffsetAPI_MakeFilling : BRepBuilderAPI_MakeShape
{
    private readonly BRepFill_Filling myFilling;

    public BRepOffsetAPI_MakeFilling(int Degree = 3, int NbPtsOnCur = 15, int NbIter = 2, bool Anisotropie = false,
                                     double Tol2d = 0.00001, double Tol3d = 0.0001, double TolAng = 0.01,
                                     double TolCurv = 0.1, int MaxDeg = 8, int MaxSegments = 9)
    {
        myFilling = new BRepFill_Filling(Degree, NbPtsOnCur, NbIter, Anisotropie, Tol2d, Tol3d, TolAng, TolCurv, MaxDeg,
                                         MaxSegments);
    }

    /// <summary>Sets the tolerances of the constraints.</summary>
    public void SetConstrParam(double Tol2d = 0.00001, double Tol3d = 0.0001, double TolAng = 0.01, double TolCurv = 0.1)
    {
        myFilling.SetConstrParam(Tol2d, Tol3d, TolAng, TolCurv);
    }

    /// <summary>Sets the parameters of the plate's resolution.</summary>
    public void SetResolParam(int Degree = 3, int NbPtsOnCur = 15, int NbIter = 2, bool Anisotropie = false)
    {
        myFilling.SetResolParam(Degree, NbPtsOnCur, NbIter, Anisotropie);
    }

    /// <summary>Sets the parameters of the approximation of the plate surface.</summary>
    public void SetApproxParam(int MaxDeg = 8, int MaxSegments = 9)
    {
        myFilling.SetApproxParam(MaxDeg, MaxSegments);
    }

    /// <summary>Loads the face whose surface the filling starts from.</summary>
    public void LoadInitSurface(in TopoDS_Face Surf)
    {
        myFilling.LoadInitSurface(Surf);
    }

    /// <summary>Adds a boundary edge (or, with <paramref name="IsBound"/> false, a free edge) with no support face.</summary>
    /// <returns>The index of the constraint.</returns>
    public int Add(in TopoDS_Edge Constr, GeomAbs_Shape Order, bool IsBound = true)
    {
        return myFilling.Add(Constr, Order, IsBound);
    }

    /// <summary>Adds an edge with its support face.</summary>
    /// <returns>The index of the constraint.</returns>
    public int Add(in TopoDS_Edge Constr, in TopoDS_Face Support, GeomAbs_Shape Order, bool IsBound = true)
    {
        return myFilling.Add(Constr, Support, Order, IsBound);
    }

    /// <summary>Adds a free constraint: a face bounding a gap in the boundary.</summary>
    /// <returns>The index of the constraint.</returns>
    public int Add(in TopoDS_Face Support, GeomAbs_Shape Order)
    {
        return myFilling.Add(Support, Order);
    }

    /// <summary>Adds a point the filling passes through.</summary>
    /// <returns>The index of the constraint.</returns>
    public int Add(in gp_Pnt Point)
    {
        return myFilling.Add(Point);
    }

    /// <summary>Adds a point constraint on a face.</summary>
    /// <returns>The index of the constraint.</returns>
    public int Add(double U, double V, in TopoDS_Face Support, GeomAbs_Shape Order)
    {
        return myFilling.Add(U, V, Support, Order);
    }

    /// <summary>Builds the filling (OCCT ignores the progress range).</summary>
    public override void Build(Message_ProgressRange theRange = default)
    {
        myFilling.Build();
        myShape = myFilling.InternalFace();
    }

    public override bool IsDone() => myFilling.IsDone();

    internal override NCollection_List<TopoDS_Shape> InternalGenerated(in TopoDS_Shape S) => myFilling.InternalGenerated(S);

    // The generated shapes are sub-shapes of the face: publishing them publishes the face (decision 414).
    private protected override void BeforePublishingLists()
    {
        myShape.TShape()?.FreezeGraph();
    }

    /// <summary>The largest distance from the boundary to the surface.</summary>
    public double G0Error() => myFilling.G0Error();

    /// <summary>The largest angle between the surface and the support faces at the boundary.</summary>
    public double G1Error() => myFilling.G1Error();

    /// <summary>The largest curvature gap between the surface and the support faces at the boundary.</summary>
    public double G2Error() => myFilling.G2Error();

    /// <summary>The largest distance between the constraint <paramref name="Index"/> and the surface.</summary>
    public double G0Error(int Index) => myFilling.G0Error(Index);

    /// <summary>The largest angle between the constraint <paramref name="Index"/> and the surface.</summary>
    public double G1Error(int Index) => myFilling.G1Error(Index);

    /// <summary>The largest curvature gap between the constraint <paramref name="Index"/> and the surface.</summary>
    public double G2Error(int Index) => myFilling.G2Error(Index);
}
