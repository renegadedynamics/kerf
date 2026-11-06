# Unit 11Q, what stays in memory: for Bento's branch

**Nothing has to change for Bento to build, and nothing it reads has changed.** Kept shapes are smaller, a STEP import's peak is lower, and every shape, mesh, volume and record reads the same. The figures are in the kernel's `docs/performance.md`, "The sixth slice".

These are the two places where Bento itself holds the memory, read from `Extensions/Bento.Cad/ShapeModels.cs`. Neither is urgent. I haven't measured Bento itself; the sizes are from what the code copies.

## A shown cell holds its mesh as well as its shape

`ShapeModels.Models` keeps a `CadModel` for each value shown, for as long as the value lives. In a chain of cells, **the meshes are most of what's kept once each step has been shown:** 4.96 of 5.42 MB for 30 cells cutting holes in a plate, and 27.8 of 28.4 MB when each also fillets, because each cell's mesh is a whole new mesh while its shape shares most of its faces with the cell before.

So the choice that matters is how many meshes stay, not their format. Dropping a model that's no longer on screen, and meshing it again if it comes back, would keep memory near the shapes' own, at the cost of meshing again as the selection moves back. `Models` could hold a few recent models strongly and the rest through a weak reference, as the viewer's comment says meshing is the slow part. That's Bento's call: it depends on how people move between cells.

## `Part` copies the mesh twice over

`Part` fills `List<float>` and `List<int>` with each face's data, growing them, and then calls `ToArray`. For a large mesh that's about three times the final size in passing, on the large object heap, and the last doubling is what counts for the peak.

- **Size the lists first.** `mesh.NodeCount * 3` for positions and normals and `mesh.TriangleCount * 3` for triangles are known before the loop, so `new float[...]` and a copy of each span are enough, with no list.
- **Each face's `Positions` and `Normals` are slices of one array for the whole mesh,** with nothing spare in it (0% measured on all the guide's examples). So with `MemoryMarshal.TryGetArray` on the first face's memory, `CadMesh` could hold those two arrays as they are, if the slices are in order and it doesn't change them. Only the triangles need new indices.

This doesn't change what's kept, only the peak while a cell is shown. Large meshes are unit 11O's, which may change `ShapeMesh` itself, so wait for that before relying on the slices.
