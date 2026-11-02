// Port of: src/DataExchange/TKDESTEP/TopoDSToStep/TopoDSToStep_MakeBrepWithVoids.hxx
// Port of: src/DataExchange/TKDESTEP/TopoDSToStep/TopoDSToStep_MakeBrepWithVoids.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings
#pragma warning disable CS0168 // locals OCCT declares and never uses

using static Occt.TopAbs_ShapeEnum;

namespace Occt;

/// <summary>Makes a brep_with_voids from a solid holding more than one closed shell.</summary>
public sealed class TopoDSToStep_MakeBrepWithVoids : TopoDSToStep_Root
{
    private StepShape_BrepWithVoids theBrepWithVoids;
    private StepVisual_TessellatedItem theTessellatedItem;

    public TopoDSToStep_MakeBrepWithVoids(in TopoDS_Solid aSolid, Transfer_FinderProcess FP, in StepData_Factors theLocalFactors,
                                          Message_ProgressRange theProgress = default)
    {
        done = false;
        TopoDS_Iterator It = new TopoDS_Iterator();
        var aMap = new NCollection_DataMap<TopoDS_Shape, Standard_Transient>(TopTools_ShapeMapHasher.Instance);
        var S = new NCollection_Sequence<Standard_Transient>();
        TopoDS_Shell aOutShell = new TopoDS_Shell();

        StepShape_TopologicalRepresentationItem aItem;
        StepShape_ClosedShell aOuter = null, aCShell;
        StepShape_OrientedClosedShell aOCShell;
        NCollection_HArray1<StepShape_OrientedClosedShell> aVoids;
        var aTessShells = new NCollection_Sequence<Standard_Transient>();

        StepData_StepModel aStepModel = FP.Model() as StepData_StepModel;
        int aWriteTessGeom = (int)aStepModel.InternalParameters.WriteTessellated;

        try
        {
            // BRepClass3d::OuterShell requires geometry when the solid has voids
            aOutShell = BRepClass3d.OuterShell(aSolid);
        }
        catch (Exception anException) when (anException is not OutOfMemoryException and not OperationCanceledException)
        {
            // OCCT: catch (...)
        }

        var StepB = new TopoDSToStep_Builder();
        var aTool = new TopoDSToStep_Tool(aStepModel);

        int nbshapes = 0;
        for (It.Initialize(aSolid); It.More(); It.Next())
        {
            if (It.Value().ShapeType() == TopAbs_SHELL)
            {
                nbshapes++;
            }
        }

        using var aPS = new Message_ProgressScope(theProgress, null, nbshapes);
        for (It.Initialize(aSolid); It.More() && aPS.More(); It.Next())
        {
            if (It.Value().ShapeType() == TopAbs_SHELL)
            {
                TopoDS_Shell CurrentShell = TopoDS.Shell(It.Value());
                if (!aOutShell.IsNull() && !aOutShell.IsEqual(CurrentShell))
                { //: e0 abv 25 Mar 98: voids should be reversed
                    //: according to EXPRESS for ABSR
                    CurrentShell.Reverse();
                }

                //: d7 abv 16 Mar 98: try to treat 'open' shells as closed since flag
                // IsClosed() is often incorrect (taken from MakeManifoldSolid(Solid))
                aTool.Init(aMap, false, aStepModel.InternalParameters.WriteSurfaceCurMode ? 1 : 0);
                StepB.Init(CurrentShell, aTool, FP, aWriteTessGeom, theLocalFactors, aPS.Next());
                TopoDSToStep.AddResult(FP, aTool);

                if (StepB.IsDone())
                {
                    aCShell = StepB.Value() as StepShape_ClosedShell;
                    // si OPEN on le force a CLOSED mais que c est une honte !
                    if (aCShell is null)
                    {
                        StepShape_OpenShell aOShell = StepB.Value() as StepShape_OpenShell;
                        if (aOShell is not null)
                        {
                            aCShell = new StepShape_ClosedShell();
                            aCShell.Init(aOShell.Name(), aOShell.CfsFaces());
                        }
                    }

                    StepVisual_TessellatedItem aTessShell = StepB.TessellatedValue();
                    if (aTessShell is not null)
                    {
                        aTessShells.Append(aTessShell);
                    }

                    if (!aOutShell.IsNull() && aCShell is not null)
                    {
                        if (aOutShell.IsEqual(It.Value()))
                        {
                            aOuter = aCShell;
                        }
                        else
                        {
                            S.Append(aCShell);
                        }
                    }
                }
            }
        }

        if (!aPS.More())
        {
            return;
        }

        int N = S.Length();
        if (N >= 1)
        {
            var aName = new TCollection_HAsciiString("");
            aVoids = new NCollection_HArray1<StepShape_OrientedClosedShell>(1, N);
            for (int i = 1; i <= N; i++)
            {
                aOCShell = new StepShape_OrientedClosedShell();
                // Warning : the Oriented Shell Orientation is not always
                //           TRUE.
                //           Shall check the TopoDS_Shell orientation.
                // => if the Shell is reversed, shall create an OrientedShell.
                aOCShell.Init(aName, S.Value(i) as StepShape_ClosedShell, false); //: e0
                aVoids.SetValue(i, aOCShell);
            }

            theBrepWithVoids = new StepShape_BrepWithVoids();
            theBrepWithVoids.Init(aName, aOuter, aVoids);
            done = true;
        }

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
            int j = 1;
            for (int k = 1; k <= aTessShells.Length(); k++)
            {
                StepVisual_TessellatedShell aTessShell = aTessShells.Value(k) as StepVisual_TessellatedShell;
                for (int i = 1; i <= aTessShell.NbItems(); ++i)
                {
                    anItems.SetValue(j++, aTessShell.ItemsValue(i));
                }
            }

            bool aHasGeomLink = theBrepWithVoids is not null;
            aTessSolid.Init(aTessName, anItems, aHasGeomLink, theBrepWithVoids);
            theTessellatedItem = aTessSolid;
            done = true;
        }
    }

    public TopoDSToStep_MakeBrepWithVoids(in TopoDS_Solid aSolid, Transfer_FinderProcess FP)
        : this(aSolid, FP, new StepData_Factors())
    {
    }

    /// <exception cref="StdFail_NotDone">The mapping wasn't done.</exception>
    public StepShape_BrepWithVoids Value()
    {
        StdFail_NotDone_Raise_if(!done, "TopoDSToStep_MakeBrepWithVoids::Value() - no result");
        return theBrepWithVoids;
    }

    /// <summary>The tessellated item made, the optional result.</summary>
    /// <exception cref="StdFail_NotDone">The mapping wasn't done.</exception>
    public StepVisual_TessellatedItem TessellatedValue()
    {
        StdFail_NotDone_Raise_if(!done, "TopoDSToStep_MakeBrepWithVoids::TessellatedValue() - no result");
        return theTessellatedItem;
    }
}
