// Port of: src/DataExchange/TKDESTEP/StepToTopoDS/StepToTopoDS_TranslateShell.hxx
// Port of: src/DataExchange/TKDESTEP/StepToTopoDS/StepToTopoDS_TranslateShell.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using static Occt.StepToTopoDS_TranslateShellError;

namespace Occt;

/// <summary>Translates a connected_face_set (an open or closed shell) or a tessellated_shell into a TopoDS_Shell.</summary>
/// <remarks><see cref="Value"/> freezes the result; the translators use <c>InternalValue</c> (decision 1259).</remarks>
public sealed class StepToTopoDS_TranslateShell : StepToTopoDS_Root
{
    private StepToTopoDS_TranslateShellError myError;
    private TopoDS_Shape myResult = new TopoDS_Shape();

    public StepToTopoDS_TranslateShell()
    {
        myError = StepToTopoDS_TranslateShellOther;
        done = false;
    }

    /// <summary>Translates a connected_face_set.</summary>
    public void Init(StepShape_ConnectedFaceSet CFS, StepToTopoDS_Tool aTool, StepToTopoDS_NMTool NMTool, in StepData_Factors theLocalFactors,
                     Message_ProgressRange theProgress = default)
    {
        // bug15697
        if (CFS is null)
        {
            return;
        }

        if (!aTool.IsBound(CFS))
        {
            var B = new BRep_Builder();
            Transfer_TransientProcess TP = aTool.TransientProcess();

            int NbFc = CFS.NbCfsFaces();
            TopoDS_Shell Sh = new TopoDS_Shell();
            B.MakeShell(ref Sh);
            TopoDS_Face F;
            TopoDS_Shape S;
            StepShape_Face StepFace;

            var myTranFace = new StepToTopoDS_TranslateFace();
            myTranFace.SetPrecision(Precision()); // gka
            myTranFace.SetMaxTol(MaxTol());

            using var PS = new Message_ProgressScope(theProgress, "Face", NbFc);
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
                        F = TopoDS.Face(S);
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

            Sh.Closed(BRep_Tool.IsClosed(Sh));
            myResult = Sh;
            aTool.Bind(CFS, myResult);
            myError = StepToTopoDS_TranslateShellDone;
            done = true;
        }
        else
        {
            myResult = TopoDS.Shell(aTool.Find(CFS));
            myError = StepToTopoDS_TranslateShellDone;
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
                     Message_ProgressRange theProgress = default)
    {
        if (theTSh is null)
        {
            return;
        }

        var aB = new BRep_Builder();
        TopoDS_Shell aSh = new TopoDS_Shell();

        int aNb = theTSh.NbItems();
        using var aPS = new Message_ProgressScope(theProgress, "Face", theTSh.HasTopologicalLink() ? aNb + 1 : aNb);

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
        if (aSh.IsNull())
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
                        aB.Add(aSh, TopoDS.Face(aTranTF.InternalValue()));
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

        aSh.Closed(BRep_Tool.IsClosed(aSh));

        myResult = aSh;
        myError = StepToTopoDS_TranslateShellDone;
        done = true;
    }

    public void Init(StepVisual_TessellatedShell theTSh, StepToTopoDS_Tool theTool, StepToTopoDS_NMTool theNMTool,
                     bool theReadTessellatedWhenNoBRepOnly, ref bool theHasGeom)
        => Init(theTSh, theTool, theNMTool, theReadTessellatedWhenNoBRepOnly, ref theHasGeom, new StepData_Factors());

    /// <summary>The shell, frozen.</summary>
    /// <exception cref="StdFail_NotDone">The translation wasn't done.</exception>
    public TopoDS_Shape Value()
    {
        TopoDS_Shape aShape = InternalValue();
        aShape.TShape()?.FreezeGraph();
        return aShape;
    }

    /// <summary>As <see cref="Value"/>, without freezing: the reader's working shape.</summary>
    internal TopoDS_Shape InternalValue()
    {
        StdFail_NotDone_Raise_if(!done, "StepToTopoDS_TranslateShell::Value() - no result");
        return myResult;
    }

    public StepToTopoDS_TranslateShellError Error() => myError;
}
