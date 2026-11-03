// Port of: src/DataExchange/TKDESTEP/StepToTopoDS/StepToTopoDS_TranslateShell.hxx
// Port of: src/DataExchange/TKDESTEP/StepToTopoDS/StepToTopoDS_TranslateShell.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Foundation;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Topology;
using static Occt.StepToTopoDS_TranslateShellError;

namespace Occt;

/// <summary>Translates a connected_face_set (an open or closed shell) or a tessellated_shell into a TopoDS_Shell.</summary>
/// <remarks><see cref="Value"/> freezes the result; the translators use <c>InternalValue</c> (decision 1247).</remarks>
public sealed class StepToTopoDS_TranslateShell : StepToTopoDS_Root
{
    private StepToTopoDS_TranslateShellError myError;
    private Shape myResult = new Shape();

    public StepToTopoDS_TranslateShell()
    {
        myError = StepToTopoDS_TranslateShellError.StepToTopoDS_TranslateShellOther;
        done = false;
    }

    /// <summary>Translates a connected_face_set.</summary>
    public void Init(StepShape_ConnectedFaceSet CFS, StepToTopoDS_Tool aTool, StepToTopoDS_NMTool NMTool, in StepData_Factors theLocalFactors,
                     ProgressRange theProgress = default)
    {
        // bug15697
        if (CFS is null)
        {
            return;
        }

        if (!aTool.IsBound(CFS))
        {
            var B = new BrepBuilder();
            Transfer_TransientProcess TP = aTool.TransientProcess();

            int NbFc = CFS.NbCfsFaces();
            Shell Sh = new Shell();
            B.MakeShell(ref Sh);
            Face F;
            Shape S;
            StepShape_Face StepFace;

            var myTranFace = new StepToTopoDS_TranslateFace();
            myTranFace.SetPrecision(Precision()); // gka
            myTranFace.SetMaxTol(MaxTol());

            using var PS = new ProgressScope(theProgress, "Face", NbFc);
            for (int i = 1; i <= NbFc && PS.More(); i++, PS.Next())
            {
                StepFace = CFS.CfsFacesValue(i);
                StepShape_FaceSurface theFS = StepFace as StepShape_FaceSurface;
                if (theFS is not null)
                {
                    myTranFace.Init(theFS, aTool, NMTool, theLocalFactors);
                    if (myTranFace.IsDone())
                    {
                        S = myTranFace.InternalValue();
                        F = ShapeCasts.Face(S);
                        B.Add(Sh, F);
                    }
                    else
                    { // Warning only + add FaceSurface file Identifier
                        TP.AddWarning(theFS, " a Face from Shell not mapped to TopoDS");
                    }
                }
                else
                { // Warning : add identifier
                    TP.AddWarning(StepFace, " Face is not of FaceSurface Type; not mapped to TopoDS");
                }
            }

            Sh.Closed(Tool.IsClosed(Sh));
            myResult = Sh;
            aTool.Bind(CFS, myResult);
            myError = StepToTopoDS_TranslateShellError.StepToTopoDS_TranslateShellDone;
            done = true;
        }
        else
        {
            myResult = ShapeCasts.Shell(aTool.Find(CFS));
            myError = StepToTopoDS_TranslateShellError.StepToTopoDS_TranslateShellDone;
            done = true;
        }
    }

    public void Init(StepShape_ConnectedFaceSet CFS, StepToTopoDS_Tool aTool, StepToTopoDS_NMTool NMTool)
        => Init(CFS, aTool, NMTool, new StepData_Factors());

    /// <summary>Translates a tessellated_shell.</summary>
    /// <remarks>
    /// With a topological link to a shell already translated, the faces' triangulations are added to that shell's
    /// faces in place (as OCCT does: they must be the reader's working shapes, unfrozen).
    /// </remarks>
    public void Init(StepVisual_TessellatedShell theTSh, StepToTopoDS_Tool theTool, StepToTopoDS_NMTool theNMTool,
                     bool theReadTessellatedWhenNoBRepOnly, ref bool theHasGeom, in StepData_Factors theLocalFactors,
                     ProgressRange theProgress = default)
    {
        if (theTSh is null)
        {
            return;
        }

        var aB = new BrepBuilder();
        Shell aSh = new Shell();

        int aNb = theTSh.NbItems();
        using var aPS = new ProgressScope(theProgress, "Face", theTSh.HasTopologicalLink() ? aNb + 1 : aNb);

        Transfer_TransientProcess aTP = theTool.TransientProcess();

        if (theTSh.HasTopologicalLink())
        {
            TransferBRep_ShapeBinder aBinder = aTP.Find(theTSh.TopologicalLink()) as TransferBRep_ShapeBinder;
            if (aBinder is not null)
            {
                aSh = aBinder.Shell();
                theHasGeom = true;
            }
        }

        bool aNewShell = false;
        if (aSh.IsNull)
        {
            aB.MakeShell(ref aSh);
            aNewShell = true;
            theHasGeom = false;
        }

        var aTranTF = new StepToTopoDS_TranslateFace();
        aTranTF.SetPrecision(Precision());
        aTranTF.SetMaxTol(MaxTol());

        for (int i = 1; i <= aNb && aPS.More(); i++, aPS.Next())
        {
            StepVisual_TessellatedStructuredItem anItem = theTSh.ItemsValue(i);
            if (anItem is StepVisual_TessellatedFace)
            {
                StepVisual_TessellatedFace aTFace = anItem as StepVisual_TessellatedFace;
                bool aHasFaceGeom = false;
                aTranTF.Init(aTFace, theTool, theNMTool, theReadTessellatedWhenNoBRepOnly, ref aHasFaceGeom, theLocalFactors);
                if (aTranTF.IsDone())
                {
                    if (aNewShell)
                    {
                        aB.Add(aSh, ShapeCasts.Face(aTranTF.InternalValue()));
                    }

                    theHasGeom &= aHasFaceGeom;
                }
                else
                {
                    aTP.AddWarning(anItem, " Triangulated face if not mapped to TopoDS");
                }
            }
            else
            {
                aTP.AddWarning(anItem, " Face is not of TriangulatedFace Type; not mapped to TopoDS");
            }
        }

        aSh.Closed(Tool.IsClosed(aSh));

        myResult = aSh;
        myError = StepToTopoDS_TranslateShellError.StepToTopoDS_TranslateShellDone;
        done = true;
    }

    public void Init(StepVisual_TessellatedShell theTSh, StepToTopoDS_Tool theTool, StepToTopoDS_NMTool theNMTool,
                     bool theReadTessellatedWhenNoBRepOnly, ref bool theHasGeom)
        => Init(theTSh, theTool, theNMTool, theReadTessellatedWhenNoBRepOnly, ref theHasGeom, new StepData_Factors());

    /// <summary>The shell, frozen.</summary>
    /// <exception cref="NotDoneException">The translation wasn't done.</exception>
    public Shape Value()
    {
        Shape aShape = InternalValue();
        aShape.TShape()?.FreezeGraph();
        return aShape;
    }

    /// <summary>As <see cref="Value"/>, without freezing: the reader's working shape.</summary>
    internal Shape InternalValue()
    {
        StdFail_NotDone_Raise_if(!done, "StepToTopoDS_TranslateShell::Value() - no result");
        return myResult;
    }

    public StepToTopoDS_TranslateShellError Error() => myError;
}
