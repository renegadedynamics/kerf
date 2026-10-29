// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepBuilderAPI/BRepBuilderAPI_Copy.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepBuilderAPI/BRepBuilderAPI_Copy.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// Duplication of a shape: a new shape with new TShapes and, if asked, copies of its geometry and triangulations
/// (<see cref="BRepTools_CopyModification"/>).
/// </summary>
public class BRepBuilderAPI_Copy : BRepBuilderAPI_ModifyShape
{
    /// <summary>An empty copy framework; give the shape with <see cref="Perform"/>.</summary>
    public BRepBuilderAPI_Copy()
    {
        myModification = new BRepTools_CopyModification(true, false);
    }

    /// <summary>Copies <paramref name="S"/>; the geometry too if <paramref name="copyGeom"/>, the triangulations if <paramref name="copyMesh"/>.</summary>
    public BRepBuilderAPI_Copy(in TopoDS_Shape S, bool copyGeom = true, bool copyMesh = false)
    {
        myModification = new BRepTools_CopyModification(copyGeom, copyMesh);
        DoModif(S);
    }

    /// <summary>
    /// Copies <paramref name="S"/>. As in OCCT, copying again a shape this framework has copied before returns the same
    /// result (the modifier keeps its images); use a new framework for a second, distinct copy.
    /// </summary>
    public void Perform(in TopoDS_Shape S, bool copyGeom = true, bool copyMesh = false)
    {
        myModification = new BRepTools_CopyModification(copyGeom, copyMesh);
        NotDone(); // on force la copie si on vient deja d`en faire une
        // OCCT: the modifier keeps the images it built, so copying a shape copied before returns the same result, with
        // only its vertices made again (occt-issues.md)
        DoModif(S);
    }
}
