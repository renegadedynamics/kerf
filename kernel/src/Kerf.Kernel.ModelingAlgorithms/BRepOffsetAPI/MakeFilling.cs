// Port of: src/ModelingAlgorithms/TKOffset/BRepOffsetAPI/BRepOffsetAPI_MakeFilling.hxx
// Port of: src/ModelingAlgorithms/TKOffset/BRepOffsetAPI/BRepOffsetAPI_MakeFilling.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Filling;
using Kerf.Engine.Foundation;

using Kerf.Geometry;
using Kerf.Topology;
using Occt;

namespace Kerf.Engine.Construction.Api;

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
internal class MakeFilling : MakeShape
{
    private readonly SurfaceFilling _filling;

    public MakeFilling(int degree = 3, int nbPtsOnCur = 15, int nbIter = 2, bool anisotropie = false,
                                     double tol2d = 0.00001, double tol3d = 0.0001, double tolAng = 0.01,
                                     double tolCurv = 0.1, int maxDeg = 8, int maxSegments = 9)
    {
        _filling = new SurfaceFilling(degree, nbPtsOnCur, nbIter, anisotropie, tol2d, tol3d, tolAng, tolCurv, maxDeg,
                                         maxSegments);
    }

    /// <summary>Sets the tolerances of the constraints.</summary>
    public void SetConstrParam(double tol2d = 0.00001, double tol3d = 0.0001, double tolAng = 0.01, double tolCurv = 0.1)
    {
        _filling.SetConstrParam(tol2d, tol3d, tolAng, tolCurv);
    }

    /// <summary>Sets the parameters of the plate's resolution.</summary>
    public void SetResolParam(int degree = 3, int nbPtsOnCur = 15, int nbIter = 2, bool anisotropie = false)
    {
        _filling.SetResolParam(degree, nbPtsOnCur, nbIter, anisotropie);
    }

    /// <summary>Sets the parameters of the approximation of the plate surface.</summary>
    public void SetApproxParam(int maxDeg = 8, int maxSegments = 9)
    {
        _filling.SetApproxParam(maxDeg, maxSegments);
    }

    /// <summary>Loads the face whose surface the filling starts from.</summary>
    public void LoadInitSurface(in Face surf)
    {
        _filling.LoadInitSurface(surf);
    }

    /// <summary>Adds a boundary edge (or, with <paramref name="isBound"/> false, a free edge) with no support face.</summary>
    /// <returns>The index of the constraint.</returns>
    public int Add(in Edge constr, Continuity order, bool isBound = true)
    {
        return _filling.Add(constr, order, isBound);
    }

    /// <summary>Adds an edge with its support face.</summary>
    /// <returns>The index of the constraint.</returns>
    public int Add(in Edge constr, in Face support, Continuity order, bool isBound = true)
    {
        return _filling.Add(constr, support, order, isBound);
    }

    /// <summary>Adds a free constraint: a face bounding a gap in the boundary.</summary>
    /// <returns>The index of the constraint.</returns>
    public int Add(in Face support, Continuity order)
    {
        return _filling.Add(support, order);
    }

    /// <summary>Adds a point the filling passes through.</summary>
    /// <returns>The index of the constraint.</returns>
    public int Add(in Point3D point)
    {
        return _filling.Add(point);
    }

    /// <summary>Adds a point constraint on a face.</summary>
    /// <returns>The index of the constraint.</returns>
    public int Add(double u, double v, in Face support, Continuity order)
    {
        return _filling.Add(u, v, support, order);
    }

    /// <summary>Builds the filling (OCCT ignores the progress range).</summary>
    public override void Build(ProgressRange range = default)
    {
        _filling.Build();
        _shape = _filling.InternalFace();
    }

    public override bool IsDone => _filling.IsDone;

    internal override NCollection_List<Shape> InternalGenerated(in Shape s) => _filling.InternalGenerated(s);

    // The generated shapes are sub-shapes of the face: publishing them publishes the face (decision 414).
    private protected override void BeforePublishingLists()
    {
        _shape.TShape()?.FreezeGraph();
    }

    /// <summary>The filling algorithm, for ported code and tests.</summary>
    internal SurfaceFilling InternalFilling() => _filling;

    /// <summary>The largest distance from the boundary to the surface.</summary>
    public double G0Error() => _filling.G0Error();

    /// <summary>The largest angle between the surface and the support faces at the boundary.</summary>
    public double G1Error() => _filling.G1Error();

    /// <summary>The largest curvature gap between the surface and the support faces at the boundary.</summary>
    public double G2Error() => _filling.G2Error();

    /// <summary>The largest distance between the constraint <paramref name="index"/> and the surface.</summary>
    public double G0Error(int index) => _filling.G0Error(index);

    /// <summary>The largest angle between the constraint <paramref name="index"/> and the surface.</summary>
    public double G1Error(int index) => _filling.G1Error(index);

    /// <summary>The largest curvature gap between the constraint <paramref name="index"/> and the surface.</summary>
    public double G2Error(int index) => _filling.G2Error(index);
}
