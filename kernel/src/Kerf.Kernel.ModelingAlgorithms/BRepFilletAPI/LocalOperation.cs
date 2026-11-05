// Port of: src/ModelingAlgorithms/TKFillet/BRepFilletAPI/BRepFilletAPI_LocalOperation.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Api;
using Kerf.Engine.Fillets.Data;
using Kerf.Topology;
using Kerf.Engine;

namespace Kerf.Engine.Fillets.Api;

/// <summary>
/// Construction of fillets on the edges of a shell: the contours (chains of edges tangent to each other) that
/// <see cref="MakeFillet"/> and <see cref="MakeChamfer"/> build on.
/// </summary>
internal abstract class LocalOperation : MakeShape
{
    /// <summary>Adds a contour in the builder (builds a contour of tangent edges).</summary>
    public abstract void Add(in Edge e);

    /// <summary>Resets the contour of index <paramref name="ic"/>: nothing is changed in the contour.</summary>
    public abstract void ResetContour(int ic);

    /// <summary>The number of contours.</summary>
    public abstract int ContourCount { get; }

    /// <summary>The index of the contour containing the edge <paramref name="e"/>, 0 if none.</summary>
    public abstract int Contour(in Edge e);

    /// <summary>The number of edges in the contour <paramref name="i"/>.</summary>
    public abstract int NbEdges(int i);

    /// <summary>The edge <paramref name="j"/> of the contour <paramref name="i"/>.</summary>
    public abstract Edge Edge(int i, int j);

    /// <summary>Removes the contour containing the edge <paramref name="e"/>.</summary>
    public abstract void Remove(in Edge e);

    /// <summary>The length of the contour <paramref name="ic"/>.</summary>
    public abstract double Length(int ic);

    /// <summary>The first vertex of the contour <paramref name="ic"/>.</summary>
    public abstract Vertex FirstVertex(int ic);

    /// <summary>The last vertex of the contour <paramref name="ic"/>.</summary>
    public abstract Vertex LastVertex(int ic);

    /// <summary>The curvilinear abscissa of the vertex <paramref name="v"/> on the contour <paramref name="ic"/>.</summary>
    public abstract double Abscissa(int ic, in Vertex v);

    /// <summary>The relative curvilinear abscissa of the vertex <paramref name="v"/> on the contour <paramref name="ic"/>.</summary>
    public abstract double RelativeAbscissa(int ic, in Vertex v);

    /// <summary>True if the contour <paramref name="ic"/> is closed and tangent.</summary>
    public abstract bool ClosedAndTangent(int ic);

    /// <summary>True if the contour <paramref name="ic"/> is closed.</summary>
    public abstract bool Closed(int ic);

    /// <summary>Resets all the fields updated by Build, but not the contours.</summary>
    public abstract void Reset();

    /// <summary>Builds the sections of the contour <paramref name="ic"/> without the surfaces (a simulation).</summary>
    public abstract void Simulate(int ic);

    /// <summary>The number of surfaces of the simulation of the contour <paramref name="ic"/>.</summary>
    public abstract int NbSurf(int ic);

    /// <summary>The sections of the surface <paramref name="IS"/> of the simulation of the contour <paramref name="ic"/>.</summary>
    public abstract NCollection_HArray1<CircSection>? Sect(int ic, int IS);
}
