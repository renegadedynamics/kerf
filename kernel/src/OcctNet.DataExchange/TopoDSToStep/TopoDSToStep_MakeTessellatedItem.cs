// Port of: src/DataExchange/TKDESTEP/TopoDSToStep/TopoDSToStep_MakeTessellatedItem.hxx
// Port of: src/DataExchange/TKDESTEP/TopoDSToStep/TopoDSToStep_MakeTessellatedItem.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2022-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using static Occt.TopAbs_ShapeEnum;

namespace Occt;

/// <summary>
/// Maps the triangulation of a face to a triangulated_face or a triangulated_surface_set, and the triangulations of a
/// shell's faces to a tessellated_shell.
/// </summary>
public sealed class TopoDSToStep_MakeTessellatedItem : TopoDSToStep_Root
{
    private StepVisual_TessellatedItem theTessellatedItem;

    // Get parameters from a TriangulatedFace or TriangulatedSurfaceSet Poly_Triangulation
    private static void InitTriangulation(Poly_Triangulation theMesh, TCollection_HAsciiString theName, StepVisual_CoordinatesList theCoordinates,
                                          NCollection_HArray1<gp_XYZ> thePoints, NCollection_HArray2<double> theNormals,
                                          NCollection_HArray1<int> theIndices, NCollection_HArray2<int> theTrias,
                                          in StepData_Factors theLocalFactors)
    {
        double aFactor = theLocalFactors.LengthFactor();
        for (int aNodeIndex = 1; aNodeIndex <= theMesh.NbNodes(); ++aNodeIndex)
        {
            thePoints.SetValue(aNodeIndex, theMesh.Node(aNodeIndex).XYZ() / aFactor);
        }

        theCoordinates.Init(theName, thePoints);

        if (!theMesh.HasNormals())
        {
            // OCCT.NET: OCCT computes the normals into the face's own triangulation; the port into a copy, so the
            // shape written never changes (decision 13009)
            theMesh = new Poly_Triangulation(theMesh);
            Poly.ComputeNormals(theMesh);
        }

        for (int aNodeIndex = 1; aNodeIndex <= theMesh.NbNodes(); ++aNodeIndex)
        {
            gp_Dir aNorm = theMesh.Normal(aNodeIndex);
            theNormals.SetValue(aNodeIndex, 1, aNorm.X());
            theNormals.SetValue(aNodeIndex, 2, aNorm.Y());
            theNormals.SetValue(aNodeIndex, 3, aNorm.Z());
        }

        for (int aNodeIndex = 1; aNodeIndex <= theMesh.NbNodes(); ++aNodeIndex)
        {
            theIndices.SetValue(aNodeIndex, aNodeIndex);
        }

        for (int aTriangleIndex = 1; aTriangleIndex <= theMesh.NbTriangles(); ++aTriangleIndex)
        {
            Poly_Triangle aT = theMesh.Triangle(aTriangleIndex);
            theTrias.SetValue(aTriangleIndex, 1, aT.Value(1));
            theTrias.SetValue(aTriangleIndex, 2, aT.Value(2));
            theTrias.SetValue(aTriangleIndex, 3, aT.Value(3));
        }
    }

    public TopoDSToStep_MakeTessellatedItem()
    {
    }

    public TopoDSToStep_MakeTessellatedItem(in TopoDS_Face theFace, TopoDSToStep_Tool theTool, Transfer_FinderProcess theFP,
                                            bool theToPreferSurfaceSet, in StepData_Factors theLocalFactors,
                                            Message_ProgressRange theProgress = default)
    {
        Init(theFace, theTool, theFP, theToPreferSurfaceSet, theLocalFactors, theProgress);
    }

    public TopoDSToStep_MakeTessellatedItem(in TopoDS_Shell theShell, TopoDSToStep_Tool theTool, Transfer_FinderProcess theFP,
                                            in StepData_Factors theLocalFactors, Message_ProgressRange theProgress = default)
    {
        Init(theShell, theTool, theFP, theLocalFactors, theProgress);
    }

    /// <summary>Creates a triangulated_face or a triangulated_surface_set from a face.</summary>
    public void Init(in TopoDS_Face theFace, TopoDSToStep_Tool theTool, Transfer_FinderProcess theFP, bool theToPreferSurfaceSet,
                     in StepData_Factors theLocalFactors, Message_ProgressRange theProgress = default)
    {
        done = false;
        if (theProgress.UserBreak())
        {
            return;
        }

        Poly_Triangulation aMesh = BRep_Tool.Triangulation(theFace, out TopLoc_Location aLoc);
        if (aMesh is null)
        {
            var anErrShape = new TransferBRep_ShapeMapper(theFace);
            theFP.AddWarning(anErrShape, " Face not mapped to TessellatedItem");
            return;
        }

        var aName = new TCollection_HAsciiString("");
        var aCoordinates = new StepVisual_CoordinatesList();
        var aPoints = new NCollection_HArray1<gp_XYZ>(1, aMesh.NbNodes());
        var aNormals = new NCollection_HArray2<double>(1, aMesh.NbNodes(), 1, 3);
        var anIndices = new NCollection_HArray1<int>(1, aMesh.NbNodes());
        var aTrias = new NCollection_HArray2<int>(1, aMesh.NbTriangles(), 1, 3);
        InitTriangulation(aMesh, aName, aCoordinates, aPoints, aNormals, anIndices, aTrias, theLocalFactors);

        bool aHasGeomLink = theTool.IsBound(theFace);
        var aGeomLink = new StepVisual_FaceOrSurface();
        if (aHasGeomLink)
        {
            StepShape_TopologicalRepresentationItem aTopoItem = theTool.Find(theFace);
            aGeomLink.SetValue(aTopoItem);
            var aTriaFace = new StepVisual_TriangulatedFace();
            aTriaFace.Init(aName, aCoordinates, aMesh.NbNodes(), aNormals, aHasGeomLink, aGeomLink, anIndices, aTrias);
            theTessellatedItem = aTriaFace;
        }
        else if (theToPreferSurfaceSet)
        {
            var aTriaSurfaceSet = new StepVisual_TriangulatedSurfaceSet();
            aTriaSurfaceSet.Init(aName, aCoordinates, aMesh.NbNodes(), aNormals, anIndices, aTrias);
            theTessellatedItem = aTriaSurfaceSet;
        }
        else
        {
            var aTriaFace = new StepVisual_TriangulatedFace();
            aTriaFace.Init(aName, aCoordinates, aMesh.NbNodes(), aNormals, aHasGeomLink, aGeomLink, anIndices, aTrias);
            theTessellatedItem = aTriaFace;
        }

        done = true;
    }

    /// <summary>Creates a tessellated_shell from a shell.</summary>
    public void Init(in TopoDS_Shell theShell, TopoDSToStep_Tool theTool, Transfer_FinderProcess theFP, in StepData_Factors theLocalFactors,
                     Message_ProgressRange theProgress = default)
    {
        done = false;
        theTessellatedItem = null;

        if (theProgress.UserBreak())
        {
            return;
        }

        var anExp = new TopExp_Explorer();
        int aNbFaces = 0;
        for (anExp.Init(theShell, TopAbs_FACE); anExp.More(); anExp.Next(), ++aNbFaces)
        {
        }

        using var aPS = new Message_ProgressScope(theProgress, null, aNbFaces);

        var aTessFaces = new NCollection_Sequence<StepVisual_TessellatedStructuredItem>();
        for (anExp.Init(theShell, TopAbs_FACE); anExp.More() && aPS.More(); anExp.Next(), aPS.Next())
        {
            TopoDS_Face aFace = TopoDS.Face(anExp.Current());
            var aMakeFace = new TopoDSToStep_MakeTessellatedItem(aFace, theTool, theFP, false, theLocalFactors, aPS.Next());
            if (aMakeFace.IsDone())
            {
                aTessFaces.Append(aMakeFace.Value() as StepVisual_TessellatedStructuredItem);
            }
        }

        if (aTessFaces.IsEmpty())
        {
            return;
        }

        var aTessShell = new StepVisual_TessellatedShell();
        var aName = new TCollection_HAsciiString("");

        var anItems = new NCollection_HArray1<StepVisual_TessellatedStructuredItem>(1, aTessFaces.Length());
        for (int anIndx = 1; anIndx <= aTessFaces.Length(); ++anIndx)
        {
            anItems.SetValue(anIndx, aTessFaces.Value(anIndx));
        }

        StepShape_ConnectedFaceSet aFaceSet = null;
        if (theTool.IsBound(theShell))
        {
            aFaceSet = theTool.Find(theShell) as StepShape_ConnectedFaceSet;
        }

        bool aHasTopoLink = aFaceSet is not null;
        aTessShell.Init(aName, anItems, aHasTopoLink, aFaceSet);

        theTessellatedItem = aTessShell;

        // TopoDSToStep::AddResult(theFP, theShell, theTessellatedItem);
        done = true;
    }

    /// <exception cref="StdFail_NotDone">The mapping wasn't done.</exception>
    public StepVisual_TessellatedItem Value()
    {
        StdFail_NotDone_Raise_if(!done, "TopoDSToStep_MakeTessellatedItem::Value() - no result");
        return theTessellatedItem;
    }
}
