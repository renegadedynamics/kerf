// Port of: src/DataExchange/TKDESTEP/TopoDSToStep/TopoDSToStep_MakeFacetedBrepAndBrepWithVoids.hxx
// Port of: src/DataExchange/TKDESTEP/TopoDSToStep/TopoDSToStep_MakeFacetedBrepAndBrepWithVoids.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings
#pragma warning disable CS0168 // locals OCCT declares and never uses

using Kerf.Engine.Foundation;
using Kerf.Engine.Topology;
using Kerf.Topology;
using static Kerf.Topology.ShapeType;

namespace Occt;

/// <summary>Makes a faceted_brep_and_brep_with_voids from a solid holding more than one closed shell.</summary>
public sealed class TopoDSToStep_MakeFacetedBrepAndBrepWithVoids : TopoDSToStep_Root
{
    private StepShape_FacetedBrepAndBrepWithVoids theFacetedBrepAndBrepWithVoids;
    private StepVisual_TessellatedItem theTessellatedItem;

    public TopoDSToStep_MakeFacetedBrepAndBrepWithVoids(in Solid aSolid, Transfer_FinderProcess FP, in StepData_Factors theLocalFactors,
                                                        ProgressRange theProgress = default)
    {
        done = false;
        Iterator It = new Iterator();
        var aMap = new NCollection_DataMap<Shape, Transient>(ShapeHasher.Instance);
        var S = new NCollection_Sequence<Transient>();
        Shell aOutShell;

        StepShape_TopologicalRepresentationItem aItem;
        StepShape_ClosedShell aOuter = null, aCShell;
        StepShape_OrientedClosedShell aOCShell;
        NCollection_HArray1<StepShape_OrientedClosedShell> aVoids;
        var aTessShells = new NCollection_Sequence<Transient>();

        aOutShell = BRepClass3d.OuterShell(aSolid);
        StepData_StepModel aStepModel = FP.Model() as StepData_StepModel;
        var StepB = new TopoDSToStep_Builder();
        var aTool = new TopoDSToStep_Tool(aStepModel);

        if (!aOutShell.IsNull)
        {
            int nbshapes = 0;
            for (It.Initialize(aSolid); It.More(); It.Next())
            {
                if (It.Value().ShapeType() == ShapeType.Shell)
                {
                    nbshapes++;
                }
            }

            using var aPS = new ProgressScope(theProgress, null, nbshapes);
            for (It.Initialize(aSolid); It.More() && aPS.More(); It.Next())
            {
                if (It.Value().ShapeType() == ShapeType.Shell)
                {
                    ProgressRange aRange = aPS.Next();
                    Shell CurrentShell = ShapeCasts.Shell(It.Value());
                    if (It.Value().Closed())
                    {
                        aTool.Init(aMap, false, aStepModel.InternalParameters.WriteSurfaceCurMode ? 1 : 0);
                        StepB.Init(CurrentShell, aTool, FP, 0 /* OCCT: false */, theLocalFactors, aRange);
                        TopoDSToStep.AddResult(FP, aTool);

                        if (StepB.IsDone())
                        {
                            aCShell = StepB.Value() as StepShape_ClosedShell;
                            if (aOutShell.IsEqual(It.Value()))
                            {
                                aOuter = aCShell;
                            }
                            else
                            {
                                S.Append(aCShell);
                            }

                            StepVisual_TessellatedItem aTessShell = StepB.TessellatedValue();
                            if (aTessShell is not null)
                            {
                                aTessShells.Append(aTessShell);
                            }
                        }
                        else
                        {
                            var errShape = new TransferBRep_ShapeMapper(CurrentShell);
                            FP.AddWarning(errShape, " Shell from Solid not mapped to FacetedBrepAndBrepWithVoids");
                        }
                    }
                    else
                    {
                        done = false;
                        var errShape = new TransferBRep_ShapeMapper(CurrentShell);
                        FP.AddWarning(errShape, " Shell from Solid not closed; not mapped to FacetedBrepAndBrepWithVoids");
                    }
                }
            }

            if (!aPS.More())
            {
                return;
            }
        }

        int N = S.Length();
        if (N >= 1)
        {
            aVoids = new NCollection_HArray1<StepShape_OrientedClosedShell>(1, N);
            var aName = new TCollection_HAsciiString("");
            for (int i = 1; i <= N; i++)
            {
                aOCShell = new StepShape_OrientedClosedShell();
                aOCShell.Init(aName, S.Value(i) as StepShape_ClosedShell, true);
                aVoids.SetValue(i, aOCShell);
            }

            theFacetedBrepAndBrepWithVoids = new StepShape_FacetedBrepAndBrepWithVoids();
            theFacetedBrepAndBrepWithVoids.Init(aName, aOuter, aVoids);

            if (!aTessShells.IsEmpty())
            {
                var aTessSolid = new StepVisual_TessellatedSolid();
                var aTessName = new TCollection_HAsciiString("");
                int aNbItems = 0;
                for (int k = 1; k <= aTessShells.Length(); k++)
                {
                    StepVisual_TessellatedShell aTessShell = aTessShells.Value(k) as StepVisual_TessellatedShell;
                    aNbItems += aTessShell.NbItems();
                }

                var anItems = new NCollection_HArray1<StepVisual_TessellatedStructuredItem>(1, aNbItems);
                for (int k = 1; k <= aTessShells.Length(); k++)
                {
                    StepVisual_TessellatedShell aTessShell = aTessShells.Value(k) as StepVisual_TessellatedShell;
                    for (int i = 1; i <= aTessShell.NbItems(); ++i)
                    {
                        anItems.SetValue(i, aTessShell.ItemsValue(i)); // OCCT: each shell's items overwrite the first ones
                    }
                }

                bool aHasGeomLink = theFacetedBrepAndBrepWithVoids is not null;
                aTessSolid.Init(aTessName, anItems, aHasGeomLink, theFacetedBrepAndBrepWithVoids);
                theTessellatedItem = aTessSolid;
            }

            done = true;
        }
        else
        {
            done = false;
            var errShape = new TransferBRep_ShapeMapper(aSolid);
            FP.AddWarning(errShape, " Solid contains no Shell to be mapped to FacetedBrepAndBrepWithVoids");
        }
    }

    public TopoDSToStep_MakeFacetedBrepAndBrepWithVoids(in Solid aSolid, Transfer_FinderProcess FP)
        : this(aSolid, FP, new StepData_Factors())
    {
    }

    /// <exception cref="NotDoneException">The mapping wasn't done.</exception>
    public StepShape_FacetedBrepAndBrepWithVoids Value()
    {
        StdFail_NotDone_Raise_if(!done, "TopoDSToStep_MakeFacetedBrepAndBrepWithVoids::Value() - no result");
        return theFacetedBrepAndBrepWithVoids;
    }

    /// <summary>The tessellated item made, the optional result.</summary>
    /// <exception cref="NotDoneException">The mapping wasn't done.</exception>
    public StepVisual_TessellatedItem TessellatedValue()
    {
        StdFail_NotDone_Raise_if(!done, "TopoDSToStep_MakeFacetedBrepAndBrepWithVoids::TessellatedValue() - no result");
        return theTessellatedItem;
    }
}
