// Port of: src/DataExchange/TKDESTEP/TopoDSToStep/TopoDSToStep.hxx
// Port of: src/DataExchange/TKDESTEP/TopoDSToStep/TopoDSToStep.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.TopoDSToStep_BuilderError;
using static Occt.TopoDSToStep_MakeEdgeError;
using static Occt.TopoDSToStep_MakeFaceError;
using static Occt.TopoDSToStep_MakeVertexError;
using static Occt.TopoDSToStep_MakeWireError;

namespace Occt;

/// <summary>
/// The mapping from OCCT shapes to AP214 shape representations. The entry points are context dependent:
/// MakeManifoldSolidBrep, MakeBrepWithVoids, MakeFacetedBrep, MakeFacetedBrepAndBrepWithVoids,
/// MakeShellBasedSurfaceModel, MakeGeometricCurveSet; each calls the Builder, and the Tool holds their common data.
/// </summary>
/// <remarks>The writers never change the shapes they are given (frozen input is fine).</remarks>
public static class TopoDSToStep
{
    public static TCollection_HAsciiString? DecodeBuilderError(TopoDSToStep_BuilderError E)
    {
        TCollection_HAsciiString? mess = null;
        switch (E)
        {
            case TopoDSToStep_BuilderDone:
                mess = new TCollection_HAsciiString("Builder Done");
                break;
            case TopoDSToStep_NoFaceMapped:
                mess = new TCollection_HAsciiString("None of the Shell Faces has been mapped");
                break;
            case TopoDSToStep_BuilderOther:
                mess = new TCollection_HAsciiString("Other Error in Builder");
                break;
        }

        return mess;
    }

    public static TCollection_HAsciiString? DecodeFaceError(TopoDSToStep_MakeFaceError E)
    {
        TCollection_HAsciiString? mess = null;
        switch (E)
        {
            case TopoDSToStep_FaceDone:
                mess = new TCollection_HAsciiString("Face Done");
                break;
            case TopoDSToStep_FaceOther:
                mess = new TCollection_HAsciiString("Other Error in Make STEP face");
                break;
            case TopoDSToStep_InfiniteFace:
                mess = new TCollection_HAsciiString("The Face has no Outer Wire");
                break;
            case TopoDSToStep_NonManifoldFace:
                mess = new TCollection_HAsciiString("The Face is Internal or External");
                break;
            case TopoDSToStep_NoWireMapped:
                mess = new TCollection_HAsciiString("None of the Face Wires has been mapped");
                break;
        }

        return mess;
    }

    public static TCollection_HAsciiString? DecodeWireError(TopoDSToStep_MakeWireError E)
    {
        TCollection_HAsciiString? mess = null;
        switch (E)
        {
            case TopoDSToStep_WireDone:
                mess = new TCollection_HAsciiString("Wire Done");
                break;
            case TopoDSToStep_WireOther:
                mess = new TCollection_HAsciiString("Other Error in Make STEP wire");
                break;
            case TopoDSToStep_NonManifoldWire:
                mess = new TCollection_HAsciiString("The Wire is Internal or External");
                break;
        }

        return mess;
    }

    public static TCollection_HAsciiString? DecodeEdgeError(TopoDSToStep_MakeEdgeError E)
    {
        TCollection_HAsciiString? mess = null;
        switch (E)
        {
            case TopoDSToStep_EdgeDone:
                mess = new TCollection_HAsciiString("Edge Done");
                break;
            case TopoDSToStep_EdgeOther:
                mess = new TCollection_HAsciiString("Other Error in Make STEP Edge");
                break;
            case TopoDSToStep_NonManifoldEdge:
                mess = new TCollection_HAsciiString("The Edge is Internal or External");
                break;
        }

        return mess;
    }

    public static TCollection_HAsciiString? DecodeVertexError(TopoDSToStep_MakeVertexError E)
    {
        TCollection_HAsciiString? mess = null;
        switch (E)
        {
            case TopoDSToStep_VertexDone:
                mess = new TCollection_HAsciiString("Vertex Done");
                break;
            case TopoDSToStep_VertexOther:
                mess = new TCollection_HAsciiString("Other Error in Make STEP Vertex");
                break;
        }

        return mess;
    }

    /// <summary>Adds an entity to the results (binders) of a shape in the finder process.</summary>
    public static void AddResult(Transfer_FinderProcess FP, in TopoDS_Shape Shape, Standard_Transient? ent)
    {
        var result = new Transfer_SimpleBinderOfTransient();
        result.SetResult(ent);

        TransferBRep_ShapeMapper mapper = TransferBRep.ShapeMapper(FP, Shape);
        Transfer_Binder? binder = FP.Find(mapper);

        if (binder is null)
        {
            FP.Bind(mapper, result);
        }
        else
        {
            binder.AddResult(result);
        }
    }

    /// <summary>
    /// Adds every entity the tool recorded to the results (binders) in the finder process, in the order the tool
    /// recorded them (OCCT: its map's hash order, decision 1251).
    /// </summary>
    public static void AddResult(Transfer_FinderProcess FP, TopoDSToStep_Tool Tool)
    {
        foreach (KeyValuePair<TopoDS_Shape, Standard_Transient> it in Tool.Map())
        {
            AddResult(FP, it.Key, it.Value);
        }
    }
}
