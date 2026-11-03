// Port of: src/ModelingAlgorithms/TKOffset/BRepOffsetAPI/BRepOffsetAPI_NormalProjection.hxx
// Port of: src/ModelingAlgorithms/TKOffset/BRepOffsetAPI/BRepOffsetAPI_NormalProjection.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1997-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Booleans.Algo;
using Kerf.Engine.Foundation;

using Kerf.Geometry;
using Kerf.Topology;
using Occt;

namespace Kerf.Engine.Construction.Api;

/// <summary>
/// The normal projection of edges and wires on a shape (<see cref="NormalProjection"/>), as a maker: give the
/// shape, <see cref="Add"/> the edges and wires, then <see cref="Build"/>. The result is a compound of edges.
/// </summary>
/// <remarks>A maker (decision 414): the inputs are only read; <see cref="MakeShape.Shape"/> freezes.</remarks>
public sealed class ApiNormalProjection : MakeShape
{
    private readonly NormalProjection _normalProjector = new();

    /// <summary>An empty algorithm; <see cref="Init"/> gives the shape to project on.</summary>
    public ApiNormalProjection()
    {
    }

    /// <summary>The projection on the shape <paramref name="s"/>.</summary>
    public ApiNormalProjection(in Shape s)
    {
        _normalProjector.Init(s);
    }

    /// <summary>Sets the shape to project on.</summary>
    public void Init(in Shape s) => _normalProjector.Init(s);

    /// <summary>Adds an edge or a wire to project.</summary>
    public void Add(in Shape toProj) => _normalProjector.Add(toProj);

    /// <summary>Sets the approximation's parameters (<see cref="NormalProjection.SetParams"/>).</summary>
    public void SetParams(double tol3D, double tol2D, Continuity internalContinuity, int maxDegree, int maxSeg) =>
        _normalProjector.SetParams(tol3D, tol2D, internalContinuity, maxDegree, maxSeg);

    /// <summary>The maximum distance between the shape and what is projected (none if negative).</summary>
    public void SetMaxDistance(double maxDist) => _normalProjector.SetMaxDistance(maxDist);

    /// <summary>Whether the projected edges are trimmed by the faces' bounds.</summary>
    public void SetLimit(bool faceBounds = true) => _normalProjector.SetLimit(faceBounds);

    /// <summary>False keeps the initial 3D curves where the projection is close enough.</summary>
    public void Compute3d(bool with3d = true) => _normalProjector.Compute3d(with3d);

    /// <summary>Builds the projection.</summary>
    public override void Build(ProgressRange range = default)
    {
        _normalProjector.Build();
        _shape = _normalProjector.InternalProjection();
        Done();
    }

    /// <summary>True if the projection was built.</summary>
    public override bool IsDone => _normalProjector.IsDone;

    /// <summary>The result (frozen).</summary>
    public Shape Projection() => _normalProjector.Projection();

    /// <summary>The face on which the edge <paramref name="e"/> of the result lies.</summary>
    public Shape Couple(in Edge e) => _normalProjector.Couple(e);

    internal override NCollection_List<Shape> InternalGenerated(in Shape s) =>
        _normalProjector.InternalGenerated(s);

    /// <summary>The edge or wire's edge projected into the edge <paramref name="e"/> of the result.</summary>
    public Shape Ancestor(in Edge e) => _normalProjector.Ancestor(e);

    /// <summary>The result as one wire, if its edges make one (<see cref="NormalProjection.BuildWire"/>).</summary>
    public bool BuildWire(NCollection_List<Shape> listOfWire) => _normalProjector.BuildWire(listOfWire);
}
