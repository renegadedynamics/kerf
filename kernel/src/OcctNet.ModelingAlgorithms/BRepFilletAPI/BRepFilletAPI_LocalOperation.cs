// Port of: src/ModelingAlgorithms/TKFillet/BRepFilletAPI/BRepFilletAPI_LocalOperation.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Topology;

namespace Occt;

/// <summary>
/// Construction of fillets on the edges of a shell: the contours (chains of edges tangent to each other) that
/// <see cref="BRepFilletAPI_MakeFillet"/> and <see cref="BRepFilletAPI_MakeChamfer"/> build on.
/// </summary>
public abstract class BRepFilletAPI_LocalOperation : BRepBuilderAPI_MakeShape
{
    /// <summary>Adds a contour in the builder (builds a contour of tangent edges).</summary>
    public abstract void Add(in Edge E);

    /// <summary>Resets the contour of index <paramref name="IC"/>: nothing is changed in the contour.</summary>
    public abstract void ResetContour(int IC);

    /// <summary>The number of contours.</summary>
    public abstract int NbContours();

    /// <summary>The index of the contour containing the edge <paramref name="E"/>, 0 if none.</summary>
    public abstract int Contour(in Edge E);

    /// <summary>The number of edges in the contour <paramref name="I"/>.</summary>
    public abstract int NbEdges(int I);

    /// <summary>The edge <paramref name="J"/> of the contour <paramref name="I"/>.</summary>
    public abstract Edge Edge(int I, int J);

    /// <summary>Removes the contour containing the edge <paramref name="E"/>.</summary>
    public abstract void Remove(in Edge E);

    /// <summary>The length of the contour <paramref name="IC"/>.</summary>
    public abstract double Length(int IC);

    /// <summary>The first vertex of the contour <paramref name="IC"/>.</summary>
    public abstract Vertex FirstVertex(int IC);

    /// <summary>The last vertex of the contour <paramref name="IC"/>.</summary>
    public abstract Vertex LastVertex(int IC);

    /// <summary>The curvilinear abscissa of the vertex <paramref name="V"/> on the contour <paramref name="IC"/>.</summary>
    public abstract double Abscissa(int IC, in Vertex V);

    /// <summary>The relative curvilinear abscissa of the vertex <paramref name="V"/> on the contour <paramref name="IC"/>.</summary>
    public abstract double RelativeAbscissa(int IC, in Vertex V);

    /// <summary>True if the contour <paramref name="IC"/> is closed and tangent.</summary>
    public abstract bool ClosedAndTangent(int IC);

    /// <summary>True if the contour <paramref name="IC"/> is closed.</summary>
    public abstract bool Closed(int IC);

    /// <summary>Resets all the fields updated by Build, but not the contours.</summary>
    public abstract void Reset();

    /// <summary>Builds the sections of the contour <paramref name="IC"/> without the surfaces (a simulation).</summary>
    public abstract void Simulate(int IC);

    /// <summary>The number of surfaces of the simulation of the contour <paramref name="IC"/>.</summary>
    public abstract int NbSurf(int IC);

    /// <summary>The sections of the surface <paramref name="IS"/> of the simulation of the contour <paramref name="IC"/>.</summary>
    public abstract NCollection_HArray1<ChFiDS_CircSection>? Sect(int IC, int IS);
}
