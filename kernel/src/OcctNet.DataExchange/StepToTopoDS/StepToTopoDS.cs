// Port of: src/DataExchange/TKDESTEP/StepToTopoDS/StepToTopoDS.hxx
// Port of: src/DataExchange/TKDESTEP/StepToTopoDS/StepToTopoDS.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.StepToTopoDS_BuilderError;
using static Occt.StepToTopoDS_GeometricToolError;
using static Occt.StepToTopoDS_TranslateEdgeError;
using static Occt.StepToTopoDS_TranslateFaceError;
using static Occt.StepToTopoDS_TranslatePolyLoopError;
using static Occt.StepToTopoDS_TranslateShellError;
using static Occt.StepToTopoDS_TranslateVertexError;
using static Occt.StepToTopoDS_TranslateVertexLoopError;

namespace Occt;

/// <summary>The texts of the STEP to topology translators' error codes.</summary>
public static class StepToTopoDS
{
    public static TCollection_HAsciiString? DecodeBuilderError(StepToTopoDS_BuilderError Error)
    {
        TCollection_HAsciiString? mess = null;
        switch (Error)
        {
            case StepToTopoDS_BuilderError.StepToTopoDS_BuilderDone:
                mess = new TCollection_HAsciiString("Builder Done");
                break;
            case StepToTopoDS_BuilderError.StepToTopoDS_BuilderOther:
                mess = new TCollection_HAsciiString("Builder Other");
                break;
        }

        return mess;
    }

    public static TCollection_HAsciiString? DecodeShellError(StepToTopoDS_TranslateShellError Error)
    {
        TCollection_HAsciiString? mess = null;
        switch (Error)
        {
            case StepToTopoDS_TranslateShellError.StepToTopoDS_TranslateShellDone:
                mess = new TCollection_HAsciiString("Translate Shell Done");
                break;
            case StepToTopoDS_TranslateShellError.StepToTopoDS_TranslateShellOther:
                mess = new TCollection_HAsciiString("Translate Shell Other");
                break;
        }

        return mess;
    }

    public static TCollection_HAsciiString? DecodeFaceError(StepToTopoDS_TranslateFaceError Error)
    {
        TCollection_HAsciiString? mess = null;
        switch (Error)
        {
            case StepToTopoDS_TranslateFaceError.StepToTopoDS_TranslateFaceDone:
                mess = new TCollection_HAsciiString("Translate Face Done");
                break;
            case StepToTopoDS_TranslateFaceError.StepToTopoDS_TranslateFaceOther:
                mess = new TCollection_HAsciiString("Translate Face Other");
                break;
        }

        return mess;
    }

    public static TCollection_HAsciiString? DecodeEdgeError(StepToTopoDS_TranslateEdgeError Error)
    {
        TCollection_HAsciiString? mess = null;
        switch (Error)
        {
            case StepToTopoDS_TranslateEdgeError.StepToTopoDS_TranslateEdgeDone:
                mess = new TCollection_HAsciiString("Translate Edge Done");
                break;
            case StepToTopoDS_TranslateEdgeError.StepToTopoDS_TranslateEdgeOther:
                mess = new TCollection_HAsciiString("Translate Edge Other");
                break;
        }

        return mess;
    }

    public static TCollection_HAsciiString? DecodeVertexError(StepToTopoDS_TranslateVertexError Error)
    {
        TCollection_HAsciiString? mess = null;
        switch (Error)
        {
            case StepToTopoDS_TranslateVertexError.StepToTopoDS_TranslateVertexDone:
                mess = new TCollection_HAsciiString("Translate Vertex Done");
                break;
            case StepToTopoDS_TranslateVertexError.StepToTopoDS_TranslateVertexOther:
                mess = new TCollection_HAsciiString("Translate Vertex Other");
                break;
        }

        return mess;
    }

    public static TCollection_HAsciiString? DecodeVertexLoopError(StepToTopoDS_TranslateVertexLoopError Error)
    {
        TCollection_HAsciiString? mess = null;
        switch (Error)
        {
            case StepToTopoDS_TranslateVertexLoopError.StepToTopoDS_TranslateVertexLoopDone:
                mess = new TCollection_HAsciiString("Translate VertexLoop Done");
                break;
            case StepToTopoDS_TranslateVertexLoopError.StepToTopoDS_TranslateVertexLoopOther:
                mess = new TCollection_HAsciiString("Translate VertexLoop Other");
                break;
        }

        return mess;
    }

    public static TCollection_HAsciiString? DecodePolyLoopError(StepToTopoDS_TranslatePolyLoopError Error)
    {
        TCollection_HAsciiString? mess = null;
        switch (Error)
        {
            case StepToTopoDS_TranslatePolyLoopError.StepToTopoDS_TranslatePolyLoopDone:
                mess = new TCollection_HAsciiString("Translate PolyLoop Done");
                break;
            case StepToTopoDS_TranslatePolyLoopError.StepToTopoDS_TranslatePolyLoopOther:
                mess = new TCollection_HAsciiString("Translate PolyLoop Other");
                break;
        }

        return mess;
    }

    public static string DecodeGeometricToolError(StepToTopoDS_GeometricToolError Error)
    {
        string mess = "";
        switch (Error)
        {
            case StepToTopoDS_GeometricToolError.StepToTopoDS_GeometricToolDone:
                mess = " Geometric Tool is done";
                break;
            case StepToTopoDS_GeometricToolError.StepToTopoDS_GeometricToolIsDegenerated:
                mess = " an Edge is degenerated";
                break;
            case StepToTopoDS_GeometricToolError.StepToTopoDS_GeometricToolHasNoPCurve:
                mess = " SurfaceCurve does not contain a PCurve lying on the BasisSurface";
                break;
            case StepToTopoDS_GeometricToolError.StepToTopoDS_GeometricToolWrong3dParameters:
                mess = " the update of 3D-Parameters failed";
                break;
            case StepToTopoDS_GeometricToolError.StepToTopoDS_GeometricToolNoProjectiOnCurve:
                mess = " the projection of a VertexPoint on the curve3d failed";
                break;
            case StepToTopoDS_GeometricToolError.StepToTopoDS_GeometricToolOther:
                mess = " GeometricTool failed";
                break;
        }

        return mess;
    }
}
