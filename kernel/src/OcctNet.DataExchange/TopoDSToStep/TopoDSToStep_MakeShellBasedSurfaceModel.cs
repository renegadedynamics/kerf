// Port of: src/DataExchange/TKDESTEP/TopoDSToStep/TopoDSToStep_MakeShellBasedSurfaceModel.hxx
// Port of: src/DataExchange/TKDESTEP/TopoDSToStep/TopoDSToStep_MakeShellBasedSurfaceModel.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using static Occt.DESTEP_Parameters.WriteMode_StepSchema;
using static Occt.TopAbs_ShapeEnum;

namespace Occt;

/// <summary>Makes a shell_based_surface_model from a face, a shell or a solid.</summary>
public sealed class TopoDSToStep_MakeShellBasedSurfaceModel : TopoDSToStep_Root
{
    private StepShape_ShellBasedSurfaceModel theShellBasedSurfaceModel;
    private StepVisual_TessellatedItem theTessellatedItem;

    /// <summary>A shell_based_surface_model of one open shell holding the face.</summary>
    public TopoDSToStep_MakeShellBasedSurfaceModel(in TopoDS_Face aFace, Transfer_FinderProcess FP, in StepData_Factors theLocalFactors,
                                                   Message_ProgressRange theProgress = default)
    {
        done = false;
        var aMap = new NCollection_DataMap<TopoDS_Shape, Standard_Transient>(TopTools_ShapeMapHasher.Instance);

        StepData_StepModel aStepModel = FP.Model() as StepData_StepModel;
        int aWriteTessGeom = (int)aStepModel.InternalParameters.WriteTessellated;
        int aWriteTessSchema = (int)aStepModel.InternalParameters.WriteSchema;
        if (aWriteTessSchema != (int)WriteMode_StepSchema_AP242DIS)
        {
            aWriteTessGeom = 0;
            var anErrShape = new TransferBRep_ShapeMapper(aFace);
            FP.AddWarning(anErrShape, " Tessellation can not be exported into not AP242");
        }

        var aTool = new TopoDSToStep_Tool(aMap, false, aStepModel.InternalParameters.WriteSurfaceCurMode ? 1 : 0);
        var StepB = new TopoDSToStep_Builder(aFace, aTool, FP, aWriteTessGeom, theLocalFactors, theProgress);
        if (theProgress.UserBreak())
        {
            return;
        }

        TopoDSToStep.AddResult(FP, aTool);

        if (StepB.IsDone())
        {
            StepShape_FaceSurface aFS = StepB.Value() as StepShape_FaceSurface;
            if (aFS is not null)
            {
                var aShellSelect = new StepShape_Shell();
                var aOpenShell = new StepShape_OpenShell();
                var aCfsFaces = new NCollection_HArray1<StepShape_Face>(1, 1);
                aCfsFaces.SetValue(1, aFS);
                var aName = new TCollection_HAsciiString("");
                aOpenShell.Init(aName, aCfsFaces);
                aShellSelect.SetValue(aOpenShell);
                var aSbsmFaces = new NCollection_HArray1<StepShape_Shell>(1, 1);
                aSbsmFaces.SetValue(1, new StepShape_Shell(aShellSelect)); // C++ stores a copy
                theShellBasedSurfaceModel = new StepShape_ShellBasedSurfaceModel();
                theShellBasedSurfaceModel.Init(aName, aSbsmFaces);
            }

            theTessellatedItem = StepB.TessellatedValue();
            done = true;
        }
        else
        {
            done = false;
            var errShape = new TransferBRep_ShapeMapper(aFace);
            FP.AddWarning(errShape, " Single Face not mapped to ShellBasedSurfaceModel");
        }
    }

    public TopoDSToStep_MakeShellBasedSurfaceModel(in TopoDS_Face aFace, Transfer_FinderProcess FP)
        : this(aFace, FP, new StepData_Factors())
    {
    }

    /// <summary>A shell_based_surface_model of the shell.</summary>
    public TopoDSToStep_MakeShellBasedSurfaceModel(in TopoDS_Shell aShell, Transfer_FinderProcess FP, in StepData_Factors theLocalFactors,
                                                   Message_ProgressRange theProgress = default)
    {
        done = false;
        var aShellSelect = new StepShape_Shell();
        NCollection_HArray1<StepShape_Shell> aSbsmBoundary;
        StepShape_OpenShell aOpenShell;
        StepShape_ClosedShell aClosedShell;
        var aMap = new NCollection_DataMap<TopoDS_Shape, Standard_Transient>(TopTools_ShapeMapHasher.Instance);

        StepData_StepModel aStepModel = FP.Model() as StepData_StepModel;
        int aWriteTessGeom = (int)aStepModel.InternalParameters.WriteTessellated;
        int aWriteTessSchema = (int)aStepModel.InternalParameters.WriteSchema;
        if (aWriteTessSchema != (int)WriteMode_StepSchema_AP242DIS)
        {
            aWriteTessGeom = 0;
            var anErrShape = new TransferBRep_ShapeMapper(aShell);
            FP.AddWarning(anErrShape, " Tessellation can not be exported into not AP242");
        }

        var aTool = new TopoDSToStep_Tool(aMap, false, aStepModel.InternalParameters.WriteSurfaceCurMode ? 1 : 0);
        var StepB = new TopoDSToStep_Builder(aShell, aTool, FP, aWriteTessGeom, theLocalFactors, theProgress);
        if (theProgress.UserBreak())
        {
            return;
        }

        // TopoDSToStep::AddResult ( FP, aTool );

        if (StepB.IsDone())
        {
            if (StepB.Value() is not null)
            {
                aSbsmBoundary = new NCollection_HArray1<StepShape_Shell>(1, 1);
                if (aShell.Closed())
                {
                    aClosedShell = StepB.Value() as StepShape_ClosedShell;
                    aShellSelect.SetValue(aClosedShell);
                }
                else
                {
                    aOpenShell = StepB.Value() as StepShape_OpenShell;
                    aShellSelect.SetValue(aOpenShell);
                }

                aSbsmBoundary.SetValue(1, new StepShape_Shell(aShellSelect)); // C++ stores a copy
                theShellBasedSurfaceModel = new StepShape_ShellBasedSurfaceModel();
                var aName = new TCollection_HAsciiString("");
                theShellBasedSurfaceModel.Init(aName, aSbsmBoundary);
                TopoDSToStep.AddResult(FP, aShell, theShellBasedSurfaceModel);
            }

            theTessellatedItem = StepB.TessellatedValue();
            done = true;
        }
        else
        {
            done = false;
            var errShape = new TransferBRep_ShapeMapper(aShell);
            FP.AddWarning(errShape, " Shell not mapped to ShellBasedSurfaceModel");
        }

        TopoDSToStep.AddResult(FP, aTool);
    }

    public TopoDSToStep_MakeShellBasedSurfaceModel(in TopoDS_Shell aShell, Transfer_FinderProcess FP)
        : this(aShell, FP, new StepData_Factors())
    {
    }

    /// <summary>A shell_based_surface_model of the solid's shells.</summary>
    public TopoDSToStep_MakeShellBasedSurfaceModel(in TopoDS_Solid aSolid, Transfer_FinderProcess FP, in StepData_Factors theLocalFactors,
                                                   Message_ProgressRange theProgress = default)
    {
        done = false;
        var aShellSelect = new StepShape_Shell();
        NCollection_HArray1<StepShape_Shell> aSbsmBoundary;
        StepShape_OpenShell aOpenShell;
        StepShape_ClosedShell aClosedShell;
        TopoDS_Iterator It = new TopoDS_Iterator();
        TopoDS_Shell aShell = new TopoDS_Shell();
        var aMap = new NCollection_DataMap<TopoDS_Shape, Standard_Transient>(TopTools_ShapeMapHasher.Instance);
        var S = new NCollection_Sequence<Standard_Transient>();
        var aTessShells = new NCollection_Sequence<Standard_Transient>();

        StepData_StepModel aStepModel = FP.Model() as StepData_StepModel;
        int aWriteTessGeom = (int)aStepModel.InternalParameters.WriteTessellated;
        int aWriteTessSchema = (int)aStepModel.InternalParameters.WriteSchema;
        if (aWriteTessSchema != (int)WriteMode_StepSchema_AP242DIS)
        {
            aWriteTessGeom = 0;
            var anErrShape = new TransferBRep_ShapeMapper(aShell); // OCCT: the shell is still null here
            FP.AddWarning(anErrShape, " Tessellation can not be exported into not AP242");
        }

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
                aShell = TopoDS.Shell(It.Value());
                var aTool = new TopoDSToStep_Tool(aMap, false, aStepModel.InternalParameters.WriteSurfaceCurMode ? 1 : 0);
                var StepB = new TopoDSToStep_Builder(aShell, aTool, FP, aWriteTessGeom, theLocalFactors, aPS.Next());
                TopoDSToStep.AddResult(FP, aTool);

                if (StepB.IsDone())
                {
                    if (StepB.Value() is not null)
                    {
                        S.Append(StepB.Value());
                    }

                    StepVisual_TessellatedItem aTessShell = StepB.TessellatedValue();
                    if (aTessShell is not null)
                    {
                        aTessShells.Append(aTessShell);
                    }
                }
                else
                {
                    var errShape = new TransferBRep_ShapeMapper(aShell);
                    FP.AddWarning(errShape, " Shell from Solid not mapped to ShellBasedSurfaceModel");
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
            aSbsmBoundary = new NCollection_HArray1<StepShape_Shell>(1, N);
            for (int i = 1; i <= N; i++)
            {
                aOpenShell = S.Value(i) as StepShape_OpenShell;
                if (aOpenShell is not null)
                {
                    aShellSelect.SetValue(aOpenShell);
                }
                else
                {
                    aClosedShell = S.Value(i) as StepShape_ClosedShell;
                    aShellSelect.SetValue(aClosedShell);
                }

                aSbsmBoundary.SetValue(i, new StepShape_Shell(aShellSelect)); // C++ stores a copy
            }

            theShellBasedSurfaceModel = new StepShape_ShellBasedSurfaceModel();
            var aName = new TCollection_HAsciiString("");
            theShellBasedSurfaceModel.Init(aName, aSbsmBoundary);

            if (!aTessShells.IsEmpty())
            {
                // OCCT makes the tessellated geometric set and drops it
                var aTessGS = new StepVisual_TessellatedGeometricSet();
                var aTessName = new TCollection_HAsciiString("");
                var anItems = new NCollection_Array1<StepVisual_TessellatedItem>(1, aTessShells.Length());
                int i = 1;
                for (int k = 1; k <= aTessShells.Length(); k++, ++i)
                {
                    StepVisual_TessellatedShell aTessShell = aTessShells.Value(k) as StepVisual_TessellatedShell;
                    anItems.SetValue(i, aTessShell);
                }

                aTessGS.Init(aTessName, anItems);
            }

            done = true;
        }
        else
        {
            done = false;
            var errShape = new TransferBRep_ShapeMapper(aSolid);
            FP.AddWarning(errShape, " Solid contains no Shell to be mapped to ShellBasedSurfaceModel");
        }
    }

    public TopoDSToStep_MakeShellBasedSurfaceModel(in TopoDS_Solid aSolid, Transfer_FinderProcess FP)
        : this(aSolid, FP, new StepData_Factors())
    {
    }

    /// <exception cref="StdFail_NotDone">The mapping wasn't done.</exception>
    public StepShape_ShellBasedSurfaceModel Value()
    {
        StdFail_NotDone_Raise_if(!done, "TopoDSToStep_MakeShellBasedSurfaceModel::Value() - no result");
        return theShellBasedSurfaceModel;
    }

    /// <summary>The tessellated item made, the optional result.</summary>
    /// <exception cref="StdFail_NotDone">The mapping wasn't done.</exception>
    public StepVisual_TessellatedItem TessellatedValue()
    {
        StdFail_NotDone_Raise_if(!done, "TopoDSToStep_MakeShellBasedSurfaceModel::TessellatedValue() - no result");
        return theTessellatedItem;
    }
}
