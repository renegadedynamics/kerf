// Port of: src/DataExchange/TKDESTEP/StepToTopoDS/StepToTopoDS.hxx
// Port of: src/DataExchange/TKDESTEP/StepToTopoDS/StepToTopoDS.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;
using static Kerf.Engine.Exchange.Step.Import.BuilderError;
using static Kerf.Engine.Exchange.Step.Import.GeometricToolError;
using static Kerf.Engine.Exchange.Step.Import.TranslateEdgeError;
using static Kerf.Engine.Exchange.Step.Import.TranslateFaceError;
using static Kerf.Engine.Exchange.Step.Import.TranslatePolyLoopError;
using static Kerf.Engine.Exchange.Step.Import.TranslateShellError;
using static Kerf.Engine.Exchange.Step.Import.TranslateVertexError;
using static Kerf.Engine.Exchange.Step.Import.TranslateVertexLoopError;

namespace Kerf.Engine.Exchange.Step.Import;

/// <summary>The texts of the STEP to topology translators' error codes.</summary>
public static class StepToTopoDS
{
    public static TCollection_HAsciiString? DecodeBuilderError(BuilderError error)
    {
        TCollection_HAsciiString? mess = null;
        switch (error)
        {
            case BuilderError.BuilderDone:
                mess = new TCollection_HAsciiString("Builder Done");
                break;
            case BuilderError.BuilderOther:
                mess = new TCollection_HAsciiString("Builder Other");
                break;
        }

        return mess;
    }

    public static TCollection_HAsciiString? DecodeShellError(TranslateShellError error)
    {
        TCollection_HAsciiString? mess = null;
        switch (error)
        {
            case TranslateShellError.TranslateShellDone:
                mess = new TCollection_HAsciiString("Translate Shell Done");
                break;
            case TranslateShellError.TranslateShellOther:
                mess = new TCollection_HAsciiString("Translate Shell Other");
                break;
        }

        return mess;
    }

    public static TCollection_HAsciiString? DecodeFaceError(TranslateFaceError error)
    {
        TCollection_HAsciiString? mess = null;
        switch (error)
        {
            case TranslateFaceError.TranslateFaceDone:
                mess = new TCollection_HAsciiString("Translate Face Done");
                break;
            case TranslateFaceError.TranslateFaceOther:
                mess = new TCollection_HAsciiString("Translate Face Other");
                break;
        }

        return mess;
    }

    public static TCollection_HAsciiString? DecodeEdgeError(TranslateEdgeError error)
    {
        TCollection_HAsciiString? mess = null;
        switch (error)
        {
            case TranslateEdgeError.TranslateEdgeDone:
                mess = new TCollection_HAsciiString("Translate Edge Done");
                break;
            case TranslateEdgeError.TranslateEdgeOther:
                mess = new TCollection_HAsciiString("Translate Edge Other");
                break;
        }

        return mess;
    }

    public static TCollection_HAsciiString? DecodeVertexError(TranslateVertexError error)
    {
        TCollection_HAsciiString? mess = null;
        switch (error)
        {
            case TranslateVertexError.TranslateVertexDone:
                mess = new TCollection_HAsciiString("Translate Vertex Done");
                break;
            case TranslateVertexError.TranslateVertexOther:
                mess = new TCollection_HAsciiString("Translate Vertex Other");
                break;
        }

        return mess;
    }

    public static TCollection_HAsciiString? DecodeVertexLoopError(TranslateVertexLoopError error)
    {
        TCollection_HAsciiString? mess = null;
        switch (error)
        {
            case TranslateVertexLoopError.TranslateVertexLoopDone:
                mess = new TCollection_HAsciiString("Translate VertexLoop Done");
                break;
            case TranslateVertexLoopError.TranslateVertexLoopOther:
                mess = new TCollection_HAsciiString("Translate VertexLoop Other");
                break;
        }

        return mess;
    }

    public static TCollection_HAsciiString? DecodePolyLoopError(TranslatePolyLoopError error)
    {
        TCollection_HAsciiString? mess = null;
        switch (error)
        {
            case TranslatePolyLoopError.TranslatePolyLoopDone:
                mess = new TCollection_HAsciiString("Translate PolyLoop Done");
                break;
            case TranslatePolyLoopError.TranslatePolyLoopOther:
                mess = new TCollection_HAsciiString("Translate PolyLoop Other");
                break;
        }

        return mess;
    }

    public static string DecodeGeometricToolError(GeometricToolError error)
    {
        string mess = "";
        switch (error)
        {
            case GeometricToolError.GeometricToolDone:
                mess = " Geometric Tool is done";
                break;
            case GeometricToolError.GeometricToolIsDegenerated:
                mess = " an Edge is degenerated";
                break;
            case GeometricToolError.GeometricToolHasNoPCurve:
                mess = " SurfaceCurve does not contain a PCurve lying on the BasisSurface";
                break;
            case GeometricToolError.GeometricToolWrong3dParameters:
                mess = " the update of 3D-Parameters failed";
                break;
            case GeometricToolError.GeometricToolNoProjectiOnCurve:
                mess = " the projection of a VertexPoint on the curve3d failed";
                break;
            case GeometricToolError.GeometricToolOther:
                mess = " GeometricTool failed";
                break;
        }

        return mess;
    }
}
