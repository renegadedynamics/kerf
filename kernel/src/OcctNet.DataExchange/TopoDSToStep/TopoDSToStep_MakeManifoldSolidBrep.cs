// Port of: src/DataExchange/TKDESTEP/TopoDSToStep/TopoDSToStep_MakeManifoldSolidBrep.hxx
// Port of: src/DataExchange/TKDESTEP/TopoDSToStep/TopoDSToStep_MakeManifoldSolidBrep.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Foundation;
using Kerf.Engine.Topology;
using Kerf.Topology;

namespace Occt;

/// <summary>Makes a manifold_solid_brep from a closed shell or from a solid's outer shell.</summary>
public sealed class TopoDSToStep_MakeManifoldSolidBrep : TopoDSToStep_Root
{
    private StepShape_ManifoldSolidBrep theManifoldSolidBrep;
    private StepVisual_TessellatedItem theTessellatedItem;

    private static (StepShape_ManifoldSolidBrep, StepVisual_TessellatedItem) MakeManifoldSolidBrep(
        in Shell aShell, Transfer_FinderProcess FP, in StepData_Factors theLocalFactors, ProgressRange theProgress)
    {
        StepShape_ManifoldSolidBrep theManifoldSolidBrep = null;
        StepVisual_TessellatedItem aTessItem = null;

        var aMap = new NCollection_DataMap<Shape, Transient>(ShapeHasher.Instance);
        StepData_StepModel aStepModel = FP.Model() as StepData_StepModel;
        var aTool = new TopoDSToStep_Tool(aMap, false, aStepModel.InternalParameters.WriteSurfaceCurMode ? 1 : 0);
        int aWriteTessGeom = (int)aStepModel.InternalParameters.WriteTessellated;

        var StepB = new TopoDSToStep_Builder(aShell, aTool, FP, aWriteTessGeom, theLocalFactors, theProgress);
        if (theProgress.UserBreak())
        {
            return (theManifoldSolidBrep, aTessItem);
        }

        TopoDSToStep.AddResult(FP, aTool);

        if (StepB.IsDone())
        {
            aTessItem = StepB.TessellatedValue();
            StepShape_ClosedShell aCShell = StepB.Value() as StepShape_ClosedShell;
            // si OPEN on le force a CLOSED mais que c est une honte !
            if (aCShell is null)
            {
                StepShape_OpenShell aOShell = StepB.Value() as StepShape_OpenShell;
                if (aOShell is null)
                {
                    return (theManifoldSolidBrep, aTessItem);
                }
                else
                {
                    aCShell = new StepShape_ClosedShell();
                    aCShell.Init(aOShell.Name(), aOShell.CfsFaces());
                }
            }

            theManifoldSolidBrep = new StepShape_ManifoldSolidBrep();
            var aName = new TCollection_HAsciiString("");
            theManifoldSolidBrep.Init(aName, aCShell);
        }

        return (theManifoldSolidBrep, aTessItem);
    }

    /// <summary>A manifold_solid_brep from a shell.</summary>
    public TopoDSToStep_MakeManifoldSolidBrep(in Shell aShell, Transfer_FinderProcess FP, in StepData_Factors theLocalFactors,
                                              ProgressRange theProgress = default)
    {
        (StepShape_ManifoldSolidBrep, StepVisual_TessellatedItem) aResult = MakeManifoldSolidBrep(aShell, FP, theLocalFactors, theProgress);
        done = aResult.Item1 is not null || aResult.Item2 is not null;
        if (done)
        {
            theManifoldSolidBrep = aResult.Item1;
            theTessellatedItem = aResult.Item2;
        }

        if (!done && !theProgress.UserBreak())
        {
            var errShape = new TransferBRep_ShapeMapper(aShell);
            FP.AddWarning(errShape, " Closed Shell not mapped to ManifoldSolidBrep");
        }
    }

    public TopoDSToStep_MakeManifoldSolidBrep(in Shell aShell, Transfer_FinderProcess FP)
        : this(aShell, FP, new StepData_Factors())
    {
    }

    /// <summary>A manifold_solid_brep from a solid holding only one closed shell.</summary>
    public TopoDSToStep_MakeManifoldSolidBrep(in Solid aSolid, Transfer_FinderProcess FP, in StepData_Factors theLocalFactors,
                                              ProgressRange theProgress = default)
    {
        Shell aOuterShell = BRepClass3d.OuterShell(aSolid);
        if (!aOuterShell.IsNull)
        {
            (StepShape_ManifoldSolidBrep, StepVisual_TessellatedItem) aResult =
                MakeManifoldSolidBrep(aOuterShell, FP, theLocalFactors, theProgress);
            done = aResult.Item1 is not null || aResult.Item2 is not null;
            if (done)
            {
                theManifoldSolidBrep = aResult.Item1;
                if (aResult.Item2 is not null)
                {
                    var aTessSolid = new StepVisual_TessellatedSolid();
                    StepVisual_TessellatedShell aTessShell = aResult.Item2 as StepVisual_TessellatedShell;
                    var aName = new TCollection_HAsciiString("");
                    var anItems = new NCollection_HArray1<StepVisual_TessellatedStructuredItem>(1, aTessShell.NbItems());
                    for (int i = 1; i <= aTessShell.NbItems(); ++i)
                    {
                        anItems.SetValue(i, aTessShell.ItemsValue(i));
                    }

                    bool aHasGeomLink = theManifoldSolidBrep is not null;
                    aTessSolid.Init(aName, anItems, aHasGeomLink, theManifoldSolidBrep);
                    theTessellatedItem = aTessSolid;
                }
            }

            if (!done && !theProgress.UserBreak())
            {
                var errShape = new TransferBRep_ShapeMapper(aOuterShell);
                FP.AddWarning(errShape, " Outer Shell of Solid not mapped to ManifoldSolidBrep");
            }
        }
        else
        {
            var errShape = new TransferBRep_ShapeMapper(aOuterShell);
            FP.AddWarning(errShape, " Outer Shell is null; not mapped to ManifoldSolidBrep ");
            done = false;
        }
    }

    public TopoDSToStep_MakeManifoldSolidBrep(in Solid aSolid, Transfer_FinderProcess FP)
        : this(aSolid, FP, new StepData_Factors())
    {
    }

    /// <exception cref="NotDoneException">The mapping wasn't done.</exception>
    public StepShape_ManifoldSolidBrep Value()
    {
        StdFail_NotDone_Raise_if(!done, "TopoDSToStep_MakeManifoldSolidBrep::Value() - no result");
        return theManifoldSolidBrep;
    }

    /// <summary>The tessellated item made, the optional result.</summary>
    /// <exception cref="NotDoneException">The mapping wasn't done.</exception>
    public StepVisual_TessellatedItem TessellatedValue()
    {
        StdFail_NotDone_Raise_if(!done, "TopoDSToStep_MakeManifoldSolidBrep::TessellatedValue() - no result");
        return theTessellatedItem;
    }
}
