// Port of: src/DataExchange/TKDESTEP/TopoDSToStep/TopoDSToStep_MakeFacetedBrep.hxx
// Port of: src/DataExchange/TKDESTEP/TopoDSToStep/TopoDSToStep_MakeFacetedBrep.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

namespace Occt;

/// <summary>Makes a faceted_brep from a closed shell or from a solid's closed outer shell.</summary>
public sealed class TopoDSToStep_MakeFacetedBrep : TopoDSToStep_Root
{
    private StepShape_FacetedBrep theFacetedBrep;
    private StepVisual_TessellatedItem theTessellatedItem;

    /// <summary>A faceted_brep from a closed shell.</summary>
    public TopoDSToStep_MakeFacetedBrep(in TopoDS_Shell aShell, Transfer_FinderProcess FP, in StepData_Factors theLocalFactors,
                                        Message_ProgressRange theProgress = default)
    {
        done = false;
        if (aShell.Closed())
        {
            StepShape_TopologicalRepresentationItem aItem;
            var aMap = new NCollection_DataMap<TopoDS_Shape, Standard_Transient>(TopTools_ShapeMapHasher.Instance);

            StepData_StepModel aStepModel = FP.Model() as StepData_StepModel;
            int aWriteTessGeom = (int)aStepModel.InternalParameters.WriteTessellated;
            var aTool = new TopoDSToStep_Tool(aMap, true, aStepModel.InternalParameters.WriteSurfaceCurMode ? 1 : 0);
            var StepB = new TopoDSToStep_Builder(aShell, aTool, FP, aWriteTessGeom, theLocalFactors, theProgress);
            if (theProgress.UserBreak())
            {
                return;
            }

            TopoDSToStep.AddResult(FP, aTool);

            if (StepB.IsDone())
            {
                aItem = StepB.Value();
                StepShape_ClosedShell aCShell;
                aCShell = aItem as StepShape_ClosedShell;
                theFacetedBrep = new StepShape_FacetedBrep();
                var aName = new TCollection_HAsciiString("");
                theFacetedBrep.Init(aName, aCShell);
                theTessellatedItem = StepB.TessellatedValue();
                done = true;
            }
            else
            {
                done = false;
                var errShape = new TransferBRep_ShapeMapper(aShell);
                FP.AddWarning(errShape, " Closed Shell not mapped to FacetedBrep");
            }
        }
        else
        {
            done = false;
            var errShape = new TransferBRep_ShapeMapper(aShell);
            FP.AddWarning(errShape, " Shell not closed; not mapped to FacetedBrep");
        }
    }

    public TopoDSToStep_MakeFacetedBrep(in TopoDS_Shell aShell, Transfer_FinderProcess FP)
        : this(aShell, FP, new StepData_Factors())
    {
    }

    /// <summary>A faceted_brep from a solid holding only one closed shell.</summary>
    /// <remarks>
    /// OCCT tests the tessellated result the wrong way round: when the builder makes none (the usual case), it reads
    /// the null tessellated shell, an access violation natively and a NullReferenceException here (occt-issues.md, 8E).
    /// </remarks>
    public TopoDSToStep_MakeFacetedBrep(in TopoDS_Solid aSolid, Transfer_FinderProcess FP, in StepData_Factors theLocalFactors,
                                        Message_ProgressRange theProgress = default)
    {
        done = false;
        // Looking for the Outer Shell
        TopoDS_Shell aOuterShell = BRepClass3d.OuterShell(aSolid);

        if (!aOuterShell.IsNull())
        {
            if (aOuterShell.Closed())
            {
                StepShape_TopologicalRepresentationItem aItem;
                var aMap = new NCollection_DataMap<TopoDS_Shape, Standard_Transient>(TopTools_ShapeMapHasher.Instance);

                StepData_StepModel aStepModel = FP.Model() as StepData_StepModel;
                int aWriteTessGeom = (int)aStepModel.InternalParameters.WriteTessellated;
                var aTool = new TopoDSToStep_Tool(aMap, true, aStepModel.InternalParameters.WriteSurfaceCurMode ? 1 : 0);
                var StepB = new TopoDSToStep_Builder(aOuterShell, aTool, FP, aWriteTessGeom, theLocalFactors, theProgress);
                if (theProgress.UserBreak())
                {
                    return;
                }

                TopoDSToStep.AddResult(FP, aTool);

                if (StepB.IsDone())
                {
                    aItem = StepB.Value();
                    StepShape_ClosedShell aCShell;
                    aCShell = aItem as StepShape_ClosedShell;
                    theFacetedBrep = new StepShape_FacetedBrep();
                    var aName = new TCollection_HAsciiString("");
                    theFacetedBrep.Init(aName, aCShell);
                    done = true;
                    if (StepB.TessellatedValue() is null) // OCCT: inverted test, reads the null tessellated shell
                    {
                        var aTessSolid = new StepVisual_TessellatedSolid();
                        StepVisual_TessellatedShell aTessShell = StepB.TessellatedValue() as StepVisual_TessellatedShell;
                        var aTessName = new TCollection_HAsciiString("");
                        var anItems = new NCollection_HArray1<StepVisual_TessellatedStructuredItem>(1, aTessShell.NbItems());
                        for (int i = 1; i <= aTessShell.NbItems(); ++i)
                        {
                            anItems.SetValue(i, aTessShell.ItemsValue(i));
                        }

                        bool aHasGeomLink = theFacetedBrep is not null;
                        aTessSolid.Init(aTessName, anItems, aHasGeomLink, theFacetedBrep);
                        theTessellatedItem = aTessSolid;
                    }
                }
                else
                {
                    done = false;
                    var errShape = new TransferBRep_ShapeMapper(aOuterShell);
                    FP.AddWarning(errShape, " Closed Outer Shell from Solid not mapped to FacetedBrep");
                }
            }
            else
            {
                done = false;
                var errShape = new TransferBRep_ShapeMapper(aOuterShell);
                FP.AddWarning(errShape, " Shell not closed; not mapped to FacetedBrep");
            }
        }
        else
        {
            done = false;
            var errShape = new TransferBRep_ShapeMapper(aOuterShell);
            FP.AddWarning(errShape, " Solid contains no Outer Shell to be mapped to FacetedBrep");
        }
    }

    public TopoDSToStep_MakeFacetedBrep(in TopoDS_Solid aSolid, Transfer_FinderProcess FP)
        : this(aSolid, FP, new StepData_Factors())
    {
    }

    /// <exception cref="StdFail_NotDone">The mapping wasn't done.</exception>
    public StepShape_FacetedBrep Value()
    {
        StdFail_NotDone_Raise_if(!done, "TopoDSToStep_MakeFacetedBrep::Value() - no result");
        return theFacetedBrep;
    }

    /// <summary>The tessellated item made, the optional result.</summary>
    /// <exception cref="StdFail_NotDone">The mapping wasn't done.</exception>
    public StepVisual_TessellatedItem TessellatedValue()
    {
        StdFail_NotDone_Raise_if(!done, "TopoDSToStep_MakeFacetedBrep::TessellatedValue() - no result");
        return theTessellatedItem;
    }
}
