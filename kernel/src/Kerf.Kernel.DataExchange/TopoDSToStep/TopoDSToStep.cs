// Port of: src/DataExchange/TKDESTEP/TopoDSToStep/TopoDSToStep.hxx
// Port of: src/DataExchange/TKDESTEP/TopoDSToStep/TopoDSToStep.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Exchange.Framework.Transfer;
using Kerf.Engine.Foundation;
using Kerf.Topology;
using Occt;
using static Kerf.Engine.Exchange.Step.Export.ExportBuilderError;
using static Kerf.Engine.Exchange.Step.Export.MakeEdgeError;
using static Kerf.Engine.Exchange.Step.Export.MakeFaceError;
using static Kerf.Engine.Exchange.Step.Export.MakeVertexError;
using static Kerf.Engine.Exchange.Step.Export.MakeWireError;

namespace Kerf.Engine.Exchange.Step.Export;

/// <summary>
/// The mapping from OCCT shapes to AP214 shape representations. The entry points are context dependent:
/// MakeManifoldSolidBrep, MakeBrepWithVoids, MakeFacetedBrep, MakeFacetedBrepAndBrepWithVoids,
/// MakeShellBasedSurfaceModel, MakeGeometricCurveSet; each calls the Builder, and the Tool holds their common data.
/// </summary>
/// <remarks>The writers never change the shapes they are given (frozen input is fine).</remarks>
internal static class TopoDSToStep
{
    public static TCollection_HAsciiString? DecodeBuilderError(ExportBuilderError e)
    {
        TCollection_HAsciiString? mess = null;
        switch (e)
        {
            case ExportBuilderError.BuilderDone:
                mess = new TCollection_HAsciiString("Builder Done");
                break;
            case ExportBuilderError.NoFaceMapped:
                mess = new TCollection_HAsciiString("None of the Shell Faces has been mapped");
                break;
            case ExportBuilderError.BuilderOther:
                mess = new TCollection_HAsciiString("Other Error in Builder");
                break;
        }

        return mess;
    }

    public static TCollection_HAsciiString? DecodeFaceError(MakeFaceError e)
    {
        TCollection_HAsciiString? mess = null;
        switch (e)
        {
            case MakeFaceError.FaceDone:
                mess = new TCollection_HAsciiString("Face Done");
                break;
            case MakeFaceError.FaceOther:
                mess = new TCollection_HAsciiString("Other Error in Make STEP face");
                break;
            case MakeFaceError.InfiniteFace:
                mess = new TCollection_HAsciiString("The Face has no Outer Wire");
                break;
            case MakeFaceError.NonManifoldFace:
                mess = new TCollection_HAsciiString("The Face is Internal or External");
                break;
            case MakeFaceError.NoWireMapped:
                mess = new TCollection_HAsciiString("None of the Face Wires has been mapped");
                break;
        }

        return mess;
    }

    public static TCollection_HAsciiString? DecodeWireError(MakeWireError e)
    {
        TCollection_HAsciiString? mess = null;
        switch (e)
        {
            case MakeWireError.WireDone:
                mess = new TCollection_HAsciiString("Wire Done");
                break;
            case MakeWireError.WireOther:
                mess = new TCollection_HAsciiString("Other Error in Make STEP wire");
                break;
            case MakeWireError.NonManifoldWire:
                mess = new TCollection_HAsciiString("The Wire is Internal or External");
                break;
        }

        return mess;
    }

    public static TCollection_HAsciiString? DecodeEdgeError(MakeEdgeError e)
    {
        TCollection_HAsciiString? mess = null;
        switch (e)
        {
            case MakeEdgeError.EdgeDone:
                mess = new TCollection_HAsciiString("Edge Done");
                break;
            case MakeEdgeError.EdgeOther:
                mess = new TCollection_HAsciiString("Other Error in Make STEP Edge");
                break;
            case MakeEdgeError.NonManifoldEdge:
                mess = new TCollection_HAsciiString("The Edge is Internal or External");
                break;
        }

        return mess;
    }

    public static TCollection_HAsciiString? DecodeVertexError(MakeVertexError e)
    {
        TCollection_HAsciiString? mess = null;
        switch (e)
        {
            case MakeVertexError.VertexDone:
                mess = new TCollection_HAsciiString("Vertex Done");
                break;
            case MakeVertexError.VertexOther:
                mess = new TCollection_HAsciiString("Other Error in Make STEP Vertex");
                break;
        }

        return mess;
    }

    /// <summary>Adds an entity to the results (binders) of a shape in the finder process.</summary>
    public static void AddResult(FinderProcess fp, in Shape shape, Transient? ent)
    {
        var result = new SimpleBinderOfTransient();
        result.SetResult(ent);

        ShapeMapper mapper = global::Kerf.Engine.Exchange.Framework.Transfer.TransferBRep.ShapeMapper(fp, shape);
        Binder? binder = fp.Find(mapper);

        if (binder is null)
        {
            fp.Bind(mapper, result);
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
    public static void AddResult(FinderProcess fp, ExportTool tool)
    {
        foreach (KeyValuePair<Shape, Transient> it in tool.Map())
        {
            AddResult(fp, it.Key, it.Value);
        }
    }
}
