// Port of: src/Visualization/TKV3d/StdPrs/StdPrs_ToolTriangulatedShape.hxx
// Port of: src/Visualization/TKV3d/StdPrs/StdPrs_ToolTriangulatedShape.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// Meshing a shape for display as OCCT's presentations do: the deflection from a <see cref="Prs3d_Drawer"/> (relative
/// to the shape's bounding box by default) and the mesh factory's algorithm, run only when the shape's mesh is
/// missing or coarser than that deflection.
/// </summary>
/// <remarks>
/// <para>
/// OCCT's <c>Tessellate</c> writes the meshes into the shape. The port's meshing algorithm meshes a shape holding
/// frozen TShapes on a copy (decision 489), so
/// <see cref="Tessellate(in TopoDS_Shape, Prs3d_Drawer, out TopoDS_Shape, out BRepTools_History)"/> also returns the
/// meshed shape and the history from the input's sub-shapes to it.
/// </para>
/// <para>
/// Not ported (unit 4L2 ports what meshing for display needs): <c>IsClosed</c>, which the shaded presentation's
/// back-face culling reads, and <c>ClearOnOwnDeflectionChange</c>, which cleans the shape in place when an object's own
/// deviation changes.
/// </para>
/// </remarks>
public static class StdPrs_ToolTriangulatedShape
{
    /// <summary>True if every face of the shape has a triangulation.</summary>
    public static bool IsTriangulated(in TopoDS_Shape theShape)
    {
        for (var aFaceIter = new TopExp_Explorer(theShape, TopAbs_ShapeEnum.TopAbs_FACE); aFaceIter.More(); aFaceIter.Next())
        {
            TopoDS_Face aFace = TopoDS.Face(aFaceIter.Current());
            if (BRep_Tool.Triangulation(aFace, out _) is null)
            {
                return false;
            }
        }

        return true;
    }

    /// <summary>
    /// The absolute deflection to mesh the shape with: the drawer's maximal chordial deviation, or with a relative
    /// type of deflection the deviation coefficient applied to the shape's bounding box (without triangulations), which
    /// is then stored into the drawer as its maximal chordial deviation.
    /// </summary>
    public static double GetDeflection(in TopoDS_Shape theShape, Prs3d_Drawer theDrawer)
    {
        if (theDrawer.TypeOfDeflection() != Aspect_TypeOfDeflection.Aspect_TOD_RELATIVE)
        {
            return theDrawer.MaximalChordialDeviation();
        }

        var aBndBox = new Bnd_Box();
        BRepBndLib.Add(theShape, ref aBndBox, false);
        if (aBndBox.IsVoid())
        {
            return theDrawer.MaximalChordialDeviation();
        }
        else if (aBndBox.IsOpen())
        {
            if (!aBndBox.HasFinitePart())
            {
                return theDrawer.MaximalChordialDeviation();
            }

            aBndBox = aBndBox.FinitePart();
        }

        // store computed relative deflection of shape as absolute deviation coefficient in case relative type to use
        // it later on for sub-shapes
        double aDeflection = Prs3d.GetDeflection(aBndBox, theDrawer.DeviationCoefficient(), theDrawer.MaximalChordialDeviation());
        theDrawer.SetMaximalChordialDeviation(aDeflection);
        return aDeflection;
    }

    /// <summary>
    /// True if the shape is meshed finely enough for the drawer: every face triangulated within
    /// <see cref="GetDeflection"/>, with polygons on its edges, and every free edge's polygon within it.
    /// </summary>
    public static bool IsTessellated(in TopoDS_Shape theShape, Prs3d_Drawer theDrawer)
        => BRepTools.Triangulation(theShape, GetDeflection(theShape, theDrawer), true);

    /// <summary>
    /// Meshes the shape with the drawer's deflection and deviation angle, through
    /// <see cref="BRepMesh_DiscretFactory"/>'s default algorithm, unless it is meshed finely enough already. True if it
    /// was meshed. An unfrozen shape is meshed in place, as in OCCT; a shape holding frozen TShapes is meshed on a
    /// copy, which only the other overload returns.
    /// </summary>
    public static bool Tessellate(in TopoDS_Shape theShape, Prs3d_Drawer theDrawer)
        => Tessellate(theShape, theDrawer, out _, out _);

    /// <summary>
    /// <see cref="Tessellate(in TopoDS_Shape, Prs3d_Drawer)"/>, with the meshed shape (the input itself when it wasn't
    /// meshed, or was meshed in place) and the history from the input's sub-shapes to the meshed shape's (empty when the
    /// algorithm isn't a <see cref="BRepMesh_IncrementalMesh"/>, or meshed in place).
    /// </summary>
    public static bool Tessellate(in TopoDS_Shape theShape, Prs3d_Drawer theDrawer, out TopoDS_Shape theMeshed,
                                  out BRepTools_History theHistory)
    {
        theMeshed = theShape;
        theHistory = new BRepTools_History();
        bool wasRecomputed = false;

        // Check if it is possible to avoid unnecessary recomputation of shape triangulation
        if (IsTessellated(theShape, theDrawer))
        {
            return wasRecomputed;
        }

        double aDeflection = GetDeflection(theShape, theDrawer);

        // retrieve meshing tool from Factory
        BRepMesh_DiscretRoot? aMeshAlgo = BRepMesh_DiscretFactory.Get().Discret(theShape, aDeflection, theDrawer.DeviationAngle());
        if (aMeshAlgo is not null)
        {
            aMeshAlgo.Perform();
            wasRecomputed = true;

            // OCCT.NET: the meshed copy (decision 489)
            theMeshed = aMeshAlgo.Shape();
            if (aMeshAlgo is BRepMesh_IncrementalMesh anIncMesh)
            {
                theHistory = anIncMesh.History();
            }
        }

        return wasRecomputed;
    }
}
