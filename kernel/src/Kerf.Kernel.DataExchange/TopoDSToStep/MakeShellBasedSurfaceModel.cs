// Port of: src/DataExchange/TKDESTEP/TopoDSToStep/TopoDSToStep_MakeShellBasedSurfaceModel.hxx
// Port of: src/DataExchange/TKDESTEP/TopoDSToStep/TopoDSToStep_MakeShellBasedSurfaceModel.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Exchange.Framework.Transfer;
using Kerf.Engine.Exchange.Step.Data;
using Kerf.Engine.Exchange.Step.Shapes;
using Kerf.Engine.Exchange.Step.Visual;
using Kerf.Engine.Foundation;
using Kerf.Engine.Topology;
using Kerf.Topology;
using Occt;
using static Kerf.Engine.Exchange.Step.Parameters.WriteMode_StepSchema;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.Exchange.Step.Export;

/// <summary>Makes a shell_based_surface_model from a face, a shell or a solid.</summary>
internal sealed class MakeShellBasedSurfaceModel : ExportRoot
{
    private StepShellBasedSurfaceModel _shellBasedSurfaceModel;
    private TessellatedItem _tessellatedItem;

    /// <summary>A shell_based_surface_model of one open shell holding the face.</summary>
    public MakeShellBasedSurfaceModel(in Face aFace, FinderProcess fp, in Factors localFactors,
                                                   ProgressRange progress = default)
    {
        _done = false;
        var map = new Dictionary<Shape, Transient>(ShapeHasher.Instance);

        StepModel stepModel = fp.Model() as StepModel;
        int writeTessGeom = (int)stepModel.InternalParameters.WriteTessellated;
        int writeTessSchema = (int)stepModel.InternalParameters.WriteSchema;
        if (writeTessSchema != (int)WriteMode_StepSchema_AP242DIS)
        {
            writeTessGeom = 0;
            var anErrShape = new ShapeMapper(aFace);
            fp.AddWarning(anErrShape, " Tessellation can not be exported into not AP242");
        }

        var tool = new ExportTool(map, false, stepModel.InternalParameters.WriteSurfaceCurMode ? 1 : 0);
        var stepB = new ExportBuilder(aFace, tool, fp, writeTessGeom, localFactors, progress);
        if (progress.UserBreak())
        {
            return;
        }

        global::Kerf.Engine.Exchange.Step.Export.TopoDSToStep.AddResult(fp, tool);

        if (stepB.IsDone)
        {
            StepFaceSurface fs = stepB.Value() as StepFaceSurface;
            if (fs is not null)
            {
                var shellSelect = new StepShell();
                var openShell = new StepOpenShell();
                var cfsFaces = new NCollection_HArray1<StepFace>(1, 1);
                cfsFaces.SetValue(1, fs);
                var name = new TCollection_HAsciiString("");
                openShell.Init(name, cfsFaces);
                shellSelect.SetValue(openShell);
                var sbsmFaces = new NCollection_HArray1<StepShell>(1, 1);
                sbsmFaces.SetValue(1, new StepShell(shellSelect)); // C++ stores a copy
                _shellBasedSurfaceModel = new StepShellBasedSurfaceModel();
                _shellBasedSurfaceModel.Init(name, sbsmFaces);
            }

            _tessellatedItem = stepB.TessellatedValue();
            _done = true;
        }
        else
        {
            _done = false;
            var errShape = new ShapeMapper(aFace);
            fp.AddWarning(errShape, " Single Face not mapped to ShellBasedSurfaceModel");
        }
    }

    public MakeShellBasedSurfaceModel(in Face aFace, FinderProcess fp)
        : this(aFace, fp, new Factors())
    {
    }

    /// <summary>A shell_based_surface_model of the shell.</summary>
    public MakeShellBasedSurfaceModel(in Shell aShell, FinderProcess fp, in Factors localFactors,
                                                   ProgressRange progress = default)
    {
        _done = false;
        var shellSelect = new StepShell();
        NCollection_HArray1<StepShell> sbsmBoundary;
        StepOpenShell openShell;
        StepClosedShell closedShell;
        var map = new Dictionary<Shape, Transient>(ShapeHasher.Instance);

        StepModel stepModel = fp.Model() as StepModel;
        int writeTessGeom = (int)stepModel.InternalParameters.WriteTessellated;
        int writeTessSchema = (int)stepModel.InternalParameters.WriteSchema;
        if (writeTessSchema != (int)WriteMode_StepSchema_AP242DIS)
        {
            writeTessGeom = 0;
            var anErrShape = new ShapeMapper(aShell);
            fp.AddWarning(anErrShape, " Tessellation can not be exported into not AP242");
        }

        var tool = new ExportTool(map, false, stepModel.InternalParameters.WriteSurfaceCurMode ? 1 : 0);
        var stepB = new ExportBuilder(aShell, tool, fp, writeTessGeom, localFactors, progress);
        if (progress.UserBreak())
        {
            return;
        }

        // TopoDSToStep::AddResult ( FP, aTool );

        if (stepB.IsDone)
        {
            if (stepB.Value() is not null)
            {
                sbsmBoundary = new NCollection_HArray1<StepShell>(1, 1);
                if (aShell.ClosedFlag)
                {
                    closedShell = stepB.Value() as StepClosedShell;
                    shellSelect.SetValue(closedShell);
                }
                else
                {
                    openShell = stepB.Value() as StepOpenShell;
                    shellSelect.SetValue(openShell);
                }

                sbsmBoundary.SetValue(1, new StepShell(shellSelect)); // C++ stores a copy
                _shellBasedSurfaceModel = new StepShellBasedSurfaceModel();
                var name = new TCollection_HAsciiString("");
                _shellBasedSurfaceModel.Init(name, sbsmBoundary);
                global::Kerf.Engine.Exchange.Step.Export.TopoDSToStep.AddResult(fp, aShell, _shellBasedSurfaceModel);
            }

            _tessellatedItem = stepB.TessellatedValue();
            _done = true;
        }
        else
        {
            _done = false;
            var errShape = new ShapeMapper(aShell);
            fp.AddWarning(errShape, " Shell not mapped to ShellBasedSurfaceModel");
        }

        global::Kerf.Engine.Exchange.Step.Export.TopoDSToStep.AddResult(fp, tool);
    }

    public MakeShellBasedSurfaceModel(in Shell aShell, FinderProcess fp)
        : this(aShell, fp, new Factors())
    {
    }

    /// <summary>A shell_based_surface_model of the solid's shells.</summary>
    public MakeShellBasedSurfaceModel(in Solid aSolid, FinderProcess fp, in Factors localFactors,
                                                   ProgressRange progress = default)
    {
        _done = false;
        var shellSelect = new StepShell();
        NCollection_HArray1<StepShell> sbsmBoundary;
        StepOpenShell openShell;
        StepClosedShell closedShell;
        Iterator it = new Iterator();
        Shell shell = new Shell();
        var map = new Dictionary<Shape, Transient>(ShapeHasher.Instance);
        var s = new NCollection_Sequence<Transient>();
        var tessShells = new NCollection_Sequence<Transient>();

        StepModel stepModel = fp.Model() as StepModel;
        int writeTessGeom = (int)stepModel.InternalParameters.WriteTessellated;
        int writeTessSchema = (int)stepModel.InternalParameters.WriteSchema;
        if (writeTessSchema != (int)WriteMode_StepSchema_AP242DIS)
        {
            writeTessGeom = 0;
            var anErrShape = new ShapeMapper(shell); // OCCT: the shell is still null here
            fp.AddWarning(anErrShape, " Tessellation can not be exported into not AP242");
        }

        int nbshapes = 0;
        for (it.Initialize(aSolid); it.More(); it.Next())
        {
            if (it.Value().Type == ShapeType.Shell)
            {
                nbshapes++;
            }
        }

        using var ps = new ProgressScope(progress, null, nbshapes);
        for (it.Initialize(aSolid); it.More() && ps.More(); it.Next())
        {
            if (it.Value().Type == ShapeType.Shell)
            {
                shell = ShapeCasts.Shell(it.Value());
                var tool = new ExportTool(map, false, stepModel.InternalParameters.WriteSurfaceCurMode ? 1 : 0);
                var stepB = new ExportBuilder(shell, tool, fp, writeTessGeom, localFactors, ps.Next());
                global::Kerf.Engine.Exchange.Step.Export.TopoDSToStep.AddResult(fp, tool);

                if (stepB.IsDone)
                {
                    if (stepB.Value() is not null)
                    {
                        s.Append(stepB.Value());
                    }

                    TessellatedItem tessShell = stepB.TessellatedValue();
                    if (tessShell is not null)
                    {
                        tessShells.Append(tessShell);
                    }
                }
                else
                {
                    var errShape = new ShapeMapper(shell);
                    fp.AddWarning(errShape, " Shell from Solid not mapped to ShellBasedSurfaceModel");
                }
            }
        }

        if (!ps.More())
        {
            return;
        }

        int n = s.Length();
        if (n >= 1)
        {
            sbsmBoundary = new NCollection_HArray1<StepShell>(1, n);
            for (int i = 1; i <= n; i++)
            {
                openShell = s.Value(i) as StepOpenShell;
                if (openShell is not null)
                {
                    shellSelect.SetValue(openShell);
                }
                else
                {
                    closedShell = s.Value(i) as StepClosedShell;
                    shellSelect.SetValue(closedShell);
                }

                sbsmBoundary.SetValue(i, new StepShell(shellSelect)); // C++ stores a copy
            }

            _shellBasedSurfaceModel = new StepShellBasedSurfaceModel();
            var name = new TCollection_HAsciiString("");
            _shellBasedSurfaceModel.Init(name, sbsmBoundary);

            if (!tessShells.IsEmpty())
            {
                // OCCT makes the tessellated geometric set and drops it
                var tessGS = new TessellatedGeometricSet();
                var tessName = new TCollection_HAsciiString("");
                var items = new NCollection_Array1<TessellatedItem>(1, tessShells.Length());
                int i = 1;
                for (int k = 1; k <= tessShells.Length(); k++, ++i)
                {
                    TessellatedShell tessShell = tessShells.Value(k) as TessellatedShell;
                    items.SetValue(i, tessShell);
                }

                tessGS.Init(tessName, items);
            }

            _done = true;
        }
        else
        {
            _done = false;
            var errShape = new ShapeMapper(aSolid);
            fp.AddWarning(errShape, " Solid contains no Shell to be mapped to ShellBasedSurfaceModel");
        }
    }

    public MakeShellBasedSurfaceModel(in Solid aSolid, FinderProcess fp)
        : this(aSolid, fp, new Factors())
    {
    }

    /// <exception cref="NotDoneException">The mapping wasn't done.</exception>
    public StepShellBasedSurfaceModel Value()
    {
        StdFail_NotDone_Raise_if(!_done, "TopoDSToStep_MakeShellBasedSurfaceModel::Value() - no result");
        return _shellBasedSurfaceModel;
    }

    /// <summary>The tessellated item made, the optional result.</summary>
    /// <exception cref="NotDoneException">The mapping wasn't done.</exception>
    public TessellatedItem TessellatedValue()
    {
        StdFail_NotDone_Raise_if(!_done, "TopoDSToStep_MakeShellBasedSurfaceModel::TessellatedValue() - no result");
        return _tessellatedItem;
    }
}
