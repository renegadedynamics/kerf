// Port of: src/DataExchange/TKDESTEP/StepToTopoDS/StepToTopoDS_TranslateSolid.hxx
// Port of: src/DataExchange/TKDESTEP/StepToTopoDS/StepToTopoDS_TranslateSolid.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2022-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Foundation;
using static Occt.StepToTopoDS_TranslateSolidError;

namespace Occt;

/// <summary>Translates a tessellated_solid into a TopoDS_Solid of faces carrying triangulations.</summary>
/// <remarks><see cref="Value"/> freezes the result; the translators use <c>InternalValue</c> (decision 1247).</remarks>
public sealed class StepToTopoDS_TranslateSolid : StepToTopoDS_Root
{
    private StepToTopoDS_TranslateSolidError myError;
    private TopoDS_Shape myResult = new TopoDS_Shape();

    public StepToTopoDS_TranslateSolid()
    {
        myError = StepToTopoDS_TranslateSolidOther;
        done = false;
    }

    public void Init(StepVisual_TessellatedSolid theTSo, Transfer_TransientProcess theTP, StepToTopoDS_Tool theTool,
                     StepToTopoDS_NMTool theNMTool, bool theReadTessellatedWhenNoBRepOnly, ref bool theHasGeom,
                     in StepData_Factors theLocalFactors, ProgressRange theProgress = default)
    {
        if (theTSo is null)
        {
            return;
        }

        var aB = new BRep_Builder();
        TopoDS_Shell aSh = new TopoDS_Shell();
        TopoDS_Solid aSo = new TopoDS_Solid();

        int aNb = theTSo.NbItems();
        using var aPS = new ProgressScope(theProgress, "Face", aNb);

        if (theTSo.HasGeometricLink() && theTP.IsBound(theTSo.GeometricLink()))
        {
            TransferBRep_ShapeBinder aBinder = theTP.Find(theTSo.GeometricLink()) as TransferBRep_ShapeBinder;
            if (aBinder is not null)
            {
                aSo = aBinder.Solid();
            }
        }

        bool aNewSolid = false;
        if (aSo.IsNull())
        {
            aB.MakeShell(ref aSh);
            aB.MakeSolid(ref aSo);
            aNewSolid = true;
            theHasGeom = false;
        }

        Transfer_TransientProcess aTP = theTool.TransientProcess();

        var aTranTF = new StepToTopoDS_TranslateFace();
        aTranTF.SetPrecision(Precision());
        aTranTF.SetMaxTol(MaxTol());

        for (int i = 1; i <= aNb && aPS.More(); i++, aPS.Next())
        {
            StepVisual_TessellatedStructuredItem anItem = theTSo.ItemsValue(i);
            if (anItem is StepVisual_TessellatedFace)
            {
                StepVisual_TessellatedFace aTFace = anItem as StepVisual_TessellatedFace;
                bool aHasFaceGeom = false;
                aTranTF.Init(aTFace, theTool, theNMTool, theReadTessellatedWhenNoBRepOnly, ref aHasFaceGeom, theLocalFactors);
                if (aTranTF.IsDone())
                {
                    if (aNewSolid)
                    {
                        aB.Add(aSh, TopoDS.Face(aTranTF.InternalValue()));
                    }

                    theHasGeom &= aHasFaceGeom;
                }
                else
                {
                    aTP.AddWarning(anItem, " Tessellated face if not mapped to TopoDS");
                }
            }
            else
            {
                aTP.AddWarning(anItem, " Face is not of TessellatedFace Type; not mapped to TopoDS");
            }
        }

        if (aNewSolid)
        {
            aB.Add(aSo, aSh);
        }

        myResult = aSo;
        myError = StepToTopoDS_TranslateSolidDone;
        done = true;
    }

    public void Init(StepVisual_TessellatedSolid theTSo, Transfer_TransientProcess theTP, StepToTopoDS_Tool theTool,
                     StepToTopoDS_NMTool theNMTool, bool theReadTessellatedWhenNoBRepOnly, ref bool theHasGeom)
        => Init(theTSo, theTP, theTool, theNMTool, theReadTessellatedWhenNoBRepOnly, ref theHasGeom, new StepData_Factors());

    /// <summary>The solid, frozen.</summary>
    /// <exception cref="NotDoneException">The translation wasn't done.</exception>
    public TopoDS_Shape Value()
    {
        TopoDS_Shape aShape = InternalValue();
        aShape.TShape()?.FreezeGraph();
        return aShape;
    }

    /// <summary>As <see cref="Value"/>, without freezing: the reader's working shape.</summary>
    internal TopoDS_Shape InternalValue()
    {
        StdFail_NotDone_Raise_if(!done, "StepToTopoDS_TranslateSolid::Value() - no result");
        return myResult;
    }

    public StepToTopoDS_TranslateSolidError Error() => myError;
}
