// Port of: src/ModelingAlgorithms/TKOffset/BRepOffsetAPI/BRepOffsetAPI_NormalProjection.hxx
// Port of: src/ModelingAlgorithms/TKOffset/BRepOffsetAPI/BRepOffsetAPI_NormalProjection.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1997-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// The normal projection of edges and wires on a shape (<see cref="BRepAlgo_NormalProjection"/>), as a maker: give the
/// shape, <see cref="Add"/> the edges and wires, then <see cref="Build"/>. The result is a compound of edges.
/// </summary>
/// <remarks>A maker (decision 414): the inputs are only read; <see cref="BRepBuilderAPI_MakeShape.Shape"/> freezes.</remarks>
public sealed class BRepOffsetAPI_NormalProjection : BRepBuilderAPI_MakeShape
{
    private readonly BRepAlgo_NormalProjection myNormalProjector = new();

    /// <summary>An empty algorithm; <see cref="Init"/> gives the shape to project on.</summary>
    public BRepOffsetAPI_NormalProjection()
    {
    }

    /// <summary>The projection on the shape <paramref name="S"/>.</summary>
    public BRepOffsetAPI_NormalProjection(in TopoDS_Shape S)
    {
        myNormalProjector.Init(S);
    }

    /// <summary>Sets the shape to project on.</summary>
    public void Init(in TopoDS_Shape S) => myNormalProjector.Init(S);

    /// <summary>Adds an edge or a wire to project.</summary>
    public void Add(in TopoDS_Shape ToProj) => myNormalProjector.Add(ToProj);

    /// <summary>Sets the approximation's parameters (<see cref="BRepAlgo_NormalProjection.SetParams"/>).</summary>
    public void SetParams(double Tol3D, double Tol2D, GeomAbs_Shape InternalContinuity, int MaxDegree, int MaxSeg) =>
        myNormalProjector.SetParams(Tol3D, Tol2D, InternalContinuity, MaxDegree, MaxSeg);

    /// <summary>The maximum distance between the shape and what is projected (none if negative).</summary>
    public void SetMaxDistance(double MaxDist) => myNormalProjector.SetMaxDistance(MaxDist);

    /// <summary>Whether the projected edges are trimmed by the faces' bounds.</summary>
    public void SetLimit(bool FaceBounds = true) => myNormalProjector.SetLimit(FaceBounds);

    /// <summary>False keeps the initial 3D curves where the projection is close enough.</summary>
    public void Compute3d(bool With3d = true) => myNormalProjector.Compute3d(With3d);

    /// <summary>Builds the projection.</summary>
    public override void Build(Message_ProgressRange theRange = default)
    {
        myNormalProjector.Build();
        myShape = myNormalProjector.InternalProjection();
        Done();
    }

    /// <summary>True if the projection was built.</summary>
    public override bool IsDone() => myNormalProjector.IsDone();

    /// <summary>The result (frozen).</summary>
    public TopoDS_Shape Projection() => myNormalProjector.Projection();

    /// <summary>The face on which the edge <paramref name="E"/> of the result lies.</summary>
    public TopoDS_Shape Couple(in TopoDS_Edge E) => myNormalProjector.Couple(E);

    internal override NCollection_List<TopoDS_Shape> InternalGenerated(in TopoDS_Shape S) =>
        myNormalProjector.InternalGenerated(S);

    /// <summary>The edge or wire's edge projected into the edge <paramref name="E"/> of the result.</summary>
    public TopoDS_Shape Ancestor(in TopoDS_Edge E) => myNormalProjector.Ancestor(E);

    /// <summary>The result as one wire, if its edges make one (<see cref="BRepAlgo_NormalProjection.BuildWire"/>).</summary>
    public bool BuildWire(NCollection_List<TopoDS_Shape> ListOfWire) => myNormalProjector.BuildWire(ListOfWire);
}
