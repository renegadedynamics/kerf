// Port of: src/DataExchange/TKDESTEP/TopoDSToStep/TopoDSToStep_MakeManifoldSolidBrep.hxx
// Port of: src/DataExchange/TKDESTEP/TopoDSToStep/TopoDSToStep_MakeManifoldSolidBrep.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
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

namespace Kerf.Engine.Exchange.Step.Export;

/// <summary>Makes a manifold_solid_brep from a closed shell or from a solid's outer shell.</summary>
internal sealed class ManifoldSolidBrepMaker : ExportRoot
{
    private StepManifoldSolidBrep _manifoldSolidBrep;
    private TessellatedItem _tessellatedItem;

    private static (StepManifoldSolidBrep, TessellatedItem) MakeManifoldSolidBrep(
        in Shell aShell, FinderProcess fp, in Factors localFactors, ProgressRange progress)
    {
        StepManifoldSolidBrep theManifoldSolidBrep = null;
        TessellatedItem tessItem = null;

        var map = new Dictionary<Shape, Transient>(ShapeHasher.Instance);
        StepModel stepModel = fp.Model() as StepModel;
        var tool = new ExportTool(map, false, stepModel.InternalParameters.WriteSurfaceCurMode ? 1 : 0);
        int writeTessGeom = (int)stepModel.InternalParameters.WriteTessellated;

        var stepB = new ExportBuilder(aShell, tool, fp, writeTessGeom, localFactors, progress);
        if (progress.UserBreak())
        {
            return (theManifoldSolidBrep, tessItem);
        }

        global::Kerf.Engine.Exchange.Step.Export.TopoDSToStep.AddResult(fp, tool);

        if (stepB.IsDone)
        {
            tessItem = stepB.TessellatedValue();
            StepClosedShell cShell = stepB.Value() as StepClosedShell;
            // si OPEN on le force a CLOSED mais que c est une honte !
            if (cShell is null)
            {
                StepOpenShell oShell = stepB.Value() as StepOpenShell;
                if (oShell is null)
                {
                    return (theManifoldSolidBrep, tessItem);
                }
                else
                {
                    cShell = new StepClosedShell();
                    cShell.Init(oShell.Name(), oShell.CfsFaces());
                }
            }

            theManifoldSolidBrep = new StepManifoldSolidBrep();
            var name = new TCollection_HAsciiString("");
            theManifoldSolidBrep.Init(name, cShell);
        }

        return (theManifoldSolidBrep, tessItem);
    }

    /// <summary>A manifold_solid_brep from a shell.</summary>
    public ManifoldSolidBrepMaker(in Shell aShell, FinderProcess fp, in Factors localFactors,
                                              ProgressRange progress = default)
    {
        (StepManifoldSolidBrep, TessellatedItem) result = MakeManifoldSolidBrep(aShell, fp, localFactors, progress);
        _done = result.Item1 is not null || result.Item2 is not null;
        if (_done)
        {
            _manifoldSolidBrep = result.Item1;
            _tessellatedItem = result.Item2;
        }

        if (!_done && !progress.UserBreak())
        {
            var errShape = new ShapeMapper(aShell);
            fp.AddWarning(errShape, " Closed Shell not mapped to ManifoldSolidBrep");
        }
    }

    public ManifoldSolidBrepMaker(in Shell aShell, FinderProcess fp)
        : this(aShell, fp, new Factors())
    {
    }

    /// <summary>A manifold_solid_brep from a solid holding only one closed shell.</summary>
    public ManifoldSolidBrepMaker(in Solid aSolid, FinderProcess fp, in Factors localFactors,
                                              ProgressRange progress = default)
    {
        Shell outerShell = global::Kerf.Engine.Classification.BRepClass3d.OuterShell(aSolid);
        if (!outerShell.IsNull)
        {
            (StepManifoldSolidBrep, TessellatedItem) result =
                MakeManifoldSolidBrep(outerShell, fp, localFactors, progress);
            _done = result.Item1 is not null || result.Item2 is not null;
            if (_done)
            {
                _manifoldSolidBrep = result.Item1;
                if (result.Item2 is not null)
                {
                    var tessSolid = new TessellatedSolid();
                    TessellatedShell tessShell = result.Item2 as TessellatedShell;
                    var name = new TCollection_HAsciiString("");
                    var items = new NCollection_HArray1<TessellatedStructuredItem>(1, tessShell.NbItems());
                    for (int i = 1; i <= tessShell.NbItems(); ++i)
                    {
                        items.SetValue(i, tessShell.ItemsValue(i));
                    }

                    bool hasGeomLink = _manifoldSolidBrep is not null;
                    tessSolid.Init(name, items, hasGeomLink, _manifoldSolidBrep);
                    _tessellatedItem = tessSolid;
                }
            }

            if (!_done && !progress.UserBreak())
            {
                var errShape = new ShapeMapper(outerShell);
                fp.AddWarning(errShape, " Outer Shell of Solid not mapped to ManifoldSolidBrep");
            }
        }
        else
        {
            var errShape = new ShapeMapper(outerShell);
            fp.AddWarning(errShape, " Outer Shell is null; not mapped to ManifoldSolidBrep ");
            _done = false;
        }
    }

    public ManifoldSolidBrepMaker(in Solid aSolid, FinderProcess fp)
        : this(aSolid, fp, new Factors())
    {
    }

    /// <exception cref="NotDoneException">The mapping wasn't done.</exception>
    public StepManifoldSolidBrep Value()
    {
        StdFail_NotDone_Raise_if(!_done, "TopoDSToStep_MakeManifoldSolidBrep::Value() - no result");
        return _manifoldSolidBrep;
    }

    /// <summary>The tessellated item made, the optional result.</summary>
    /// <exception cref="NotDoneException">The mapping wasn't done.</exception>
    public TessellatedItem TessellatedValue()
    {
        StdFail_NotDone_Raise_if(!_done, "TopoDSToStep_MakeManifoldSolidBrep::TessellatedValue() - no result");
        return _tessellatedItem;
    }
}
