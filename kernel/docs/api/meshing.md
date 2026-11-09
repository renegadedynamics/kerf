# API review: meshing

*Unit 10D, draft. Namespace `Kerf.Meshing`, in ModelingAlgorithms (the mesh types) and Foundation's Poly as engine.
Engine: `Kerf.Engine.Meshing` (BRepMesh, IMeshData, IMeshTools, BRepMeshData, DelaBella, Poly).*

## Main types

New: `Mesher`, `MeshOptions`, `ShapeMesh`, `FaceMesh`, `EdgePolyline`, `Triangle`.

```csharp
public static class Mesher   // extension member: shape.Mesh(options)
{
    public static Result<ShapeMesh> Mesh(Shape shape, MeshOptions? options = null,
                                         IProgress<double>? progress = null, CancellationToken cancellationToken = default);
}

public sealed record MeshOptions
{
    public static MeshOptions Default { get; } = new();
    public double LinearDeflection { get; init; } = 1e-3;
    public double AngularDeflection { get; init; } = 0.5;       // radians
    public bool Relative { get; init; }                         // deflection relative to each edge's size
    public bool Parallel { get; init; } = true;
    public double MinSize { get; init; }
    public long MaxTriangles { get; init; } = 10_000_000;     // more fails with meshing.too-large
    public bool InternalVertices { get; init; } = true;
}

/// <summary>A shape's mesh in flat buffers: what viewers and exporters read.</summary>
public sealed class ShapeMesh
{
    public Shape Shape { get; }
    // every face's nodes, normals and triangles, concatenated; triangles index the whole buffer, 0-based
    public ReadOnlyMemory<float> Positions { get; }             // x, y, z per node
    public ReadOnlyMemory<float> Normals { get; }
    public ReadOnlyMemory<int> Triangles { get; }               // three indices per triangle
    public ReadOnlyMemory<float> EdgePoints { get; }            // every edge's polyline, x, y, z per point
    public IReadOnlyList<FaceMesh> Faces { get; }               // one per unique face, in Shape.Faces() order
    public IReadOnlyList<EdgePolyline> Edges { get; }           // one per unique edge
    public int NodeCount { get; }  public int TriangleCount { get; }
}

public readonly struct FaceMesh                                  // a view into the shape mesh, and the exact data
{
    public Face Face { get; }
    public ReadOnlyMemory<float> Positions { get; }  public ReadOnlyMemory<float> Normals { get; }
    public ReadOnlyMemory<int> Triangles { get; }               // indices local to this face, 0-based
    public int TriangleCount { get; }
    public ReadOnlySpan<Point3D> Nodes { get; }                 // double precision, the face's placement applied
    public ReadOnlySpan<Point2D> UVNodes { get; }               // empty for a face without a surface
    public double Deflection { get; }
}

public readonly struct EdgePolyline { public Edge Edge { get; } public ReadOnlyMemory<float> Points { get; } public int PointCount { get; } }
public readonly record struct Triangle(int A, int B, int C);   // 0-based
```

- **Meshing doesn't change the shape** (question 59). OCCT's `BRepMesh_IncrementalMesh` stores triangulations in the
  faces it meshes, so the modelling layer meshes a copy today; `Mesh` returns the mesh beside the shape instead. The
  engine meshes inside the operation's working overlay (plan 2.4) and reads the triangulations out.
- **Stored meshes** (an STL face, a BREP file with triangles) are read with `Face.TryGetMesh` (topology.md), and a face
  is made from one with `Face.FromMesh(FaceMesh)`, replacing the modelling layer's `UnsafeAccessor` to
  `BRep_Builder.MakeFace(face, triangulation)`.
- The flat buffers are the ones `MeshExtraction`/`MeshBuffers` build today for build123d.NET and Bento's viewer;
  `ShapeModels.Part` in Bento reads `Positions`, `Normals`, `Triangles` per face and `Points` per edge exactly so.

## What a mesh reports

`Mesh` fails with `meshing.failed` (no face could be meshed: the failure carries the edges' polylines),
`meshing.too-large` (past `MaxTriangles`) or `meshing.faces-refused` (the failure carries the other faces' mesh). It
succeeds with these warnings:

- `meshing.faces-not-meshed` (`Mesher.FacesNotMeshed`): some faces have no triangles.
- `meshing.deflection-exceeded` (`Mesher.DeflectionExceeded`, decision 26901): some faces are further from their
  surfaces than the deflection. Spheres and tori are measured as they're meshed and refined until they keep to it; a face
  that can't be refined further is left as it is, and the warning names it: `Shapes` holds the faces, and the message
  gives how many, their indices in `ShapeMesh.Faces` (the first ten), and the furthest one's distance from its surface
  in model units and as a share of its deflection (at a relative deflection, the one relative mode works out for that
  face). The figures are the mesher's own measure, a point a triangle, which finds the misses over 3.5% and can miss
  smaller ones. Met only at relative deflections, on small fillet tori beside a long boundary edge, up to 1.14 times out
  (decisions 2056, 2057); other kinds of surface aren't measured, so it says nothing of them. The mesh is the same with
  or without it.

```csharp
Result<ShapeMesh> meshed = part.Mesh(new MeshOptions { LinearDeflection = 0.005, Relative = true });
foreach (Diagnostic d in meshed.Diagnostics.Where(d => d.Code == Mesher.DeflectionExceeded))
    Console.WriteLine($"{d.Message} ({d.Shapes.Length} faces)");
```

The writers that mesh a model (`Gltf.Write`, `ThreeMf.Write`, `Obj.Write`) give the same warning in their results, one
for each part with such faces, naming the part (`Face 2 of knob is …`). `Render` of a shape and `ModelNode.ToScene`
pass the mesh's warnings on.

## Examples

```csharp
ShapeMesh mesh = part.Mesh(new MeshOptions { LinearDeflection = 0.01, AngularDeflection = 0.2 }).Value;
gpu.Upload(mesh.Positions.Span, mesh.Normals.Span, mesh.Triangles.Span);
```

```csharp
foreach (FaceMesh face in mesh.Faces)
    perFaceCounts.Add(face.TriangleCount);
```

```csharp
double meshArea = 0;
foreach (FaceMesh face in mesh.Faces) meshArea += Area(face.Nodes, face.Triangles.Span);
Debug.Assert(Math.Abs(meshArea - part.IntegrateArea()) < 1e-2 * part.IntegrateArea());   // plan 3.2's invariant
```

```csharp
Face fromStl = Face.FromMesh(stlMesh);            // a mesh-only face, as Stl.Read returns
```

## From OCCT.NET

| Today | Kerf |
|---|---|
| `new BRepMesh_IncrementalMesh(copy, deflection, relative, angle, parallel)` | `Mesher.Mesh(shape, MeshOptions)` |
| `BRep_Tool.Triangulation(face, location)`, `Poly_Triangulation` (`NbNodes`, `Node(i)`, `Triangle(i)`, 1-based) | `FaceMesh` (`Nodes`, `Triangles`, 0-based) |
| `Poly_Triangle` (1-based) | `Triangle` (0-based) |
| `MeshExtraction.Extract`, `MeshBuffers` (`Kerf.Engine.Meshing`) | `ShapeMesh` |
| `BRep_Tool.Polygon3D`, `PolygonOnTriangulation` | `EdgePolyline` |
| `BRep_Builder.MakeFace(face, triangulation)` (an `UnsafeAccessor`) | `Face.FromMesh` |
| `IMeshTools_Parameters` | `MeshOptions` |

**Disappears:** `BRepMesh_DiscretRoot`, `IMeshTools_Context`, the mesher's plugin factory, `Poly_Connect` and
`Poly_CoherentTriangulation` from the public view.

## Open points

- `float` buffers suit viewers and file formats; `FaceMesh.Nodes` keeps `double` for analysis. If memory matters, the
  double nodes can be computed on demand rather than kept.
- One `ShapeMesh` per call: no cache on the shape. A cache keyed by `(TShape, MeshOptions)` could come later, measured.
