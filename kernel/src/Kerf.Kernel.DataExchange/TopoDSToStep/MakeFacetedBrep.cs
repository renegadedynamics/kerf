// Port of: src/DataExchange/TKDESTEP/TopoDSToStep/TopoDSToStep_MakeFacetedBrep.hxx
// Port of: src/DataExchange/TKDESTEP/TopoDSToStep/TopoDSToStep_MakeFacetedBrep.cxx
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

namespace Kerf.Engine.Exchange.Step.Export;

/// <summary>Makes a faceted_brep from a closed shell or from a solid's closed outer shell.</summary>
internal sealed class MakeFacetedBrep : ExportRoot
{
    private StepFacetedBrep _facetedBrep;
    private TessellatedItem _tessellatedItem;

    /// <summary>A faceted_brep from a closed shell.</summary>
    public MakeFacetedBrep(in Shell aShell, FinderProcess fp, in Factors localFactors,
                                        ProgressRange progress = default)
    {
        _done = false;
        if (aShell.ClosedFlag)
        {
            StepTopologicalRepresentationItem item;
            var map = new Dictionary<Shape, Transient>(ShapeHasher.Instance);

            StepModel stepModel = fp.Model() as StepModel;
            int writeTessGeom = (int)stepModel.InternalParameters.WriteTessellated;
            var tool = new ExportTool(map, true, stepModel.InternalParameters.WriteSurfaceCurMode ? 1 : 0);
            var stepB = new ExportBuilder(aShell, tool, fp, writeTessGeom, localFactors, progress);
            if (progress.UserBreak())
            {
                return;
            }

            global::Kerf.Engine.Exchange.Step.Export.TopoDSToStep.AddResult(fp, tool);

            if (stepB.IsDone)
            {
                item = stepB.Value();
                StepClosedShell cShell;
                cShell = item as StepClosedShell;
                _facetedBrep = new StepFacetedBrep();
                var name = new TCollection_HAsciiString("");
                _facetedBrep.Init(name, cShell);
                _tessellatedItem = stepB.TessellatedValue();
                _done = true;
            }
            else
            {
                _done = false;
                var errShape = new ShapeMapper(aShell);
                fp.AddWarning(errShape, " Closed Shell not mapped to FacetedBrep");
            }
        }
        else
        {
            _done = false;
            var errShape = new ShapeMapper(aShell);
            fp.AddWarning(errShape, " Shell not closed; not mapped to FacetedBrep");
        }
    }

    public MakeFacetedBrep(in Shell aShell, FinderProcess fp)
        : this(aShell, fp, new Factors())
    {
    }

    /// <summary>A faceted_brep from a solid holding only one closed shell.</summary>
    /// <remarks>
    /// OCCT tests the tessellated result the wrong way round: when the builder makes none (the usual case), it reads
    /// the null tessellated shell, an access violation natively and a NullReferenceException here (occt-issues.md, 8E).
    /// </remarks>
    public MakeFacetedBrep(in Solid aSolid, FinderProcess fp, in Factors localFactors,
                                        ProgressRange progress = default)
    {
        _done = false;
        // Looking for the Outer Shell
        Shell outerShell = global::Kerf.Engine.Classification.BRepClass3d.OuterShell(aSolid);

        if (!outerShell.IsNull)
        {
            if (outerShell.ClosedFlag)
            {
                StepTopologicalRepresentationItem item;
                var map = new Dictionary<Shape, Transient>(ShapeHasher.Instance);

                StepModel stepModel = fp.Model() as StepModel;
                int writeTessGeom = (int)stepModel.InternalParameters.WriteTessellated;
                var tool = new ExportTool(map, true, stepModel.InternalParameters.WriteSurfaceCurMode ? 1 : 0);
                var stepB = new ExportBuilder(outerShell, tool, fp, writeTessGeom, localFactors, progress);
                if (progress.UserBreak())
                {
                    return;
                }

                global::Kerf.Engine.Exchange.Step.Export.TopoDSToStep.AddResult(fp, tool);

                if (stepB.IsDone)
                {
                    item = stepB.Value();
                    StepClosedShell cShell;
                    cShell = item as StepClosedShell;
                    _facetedBrep = new StepFacetedBrep();
                    var name = new TCollection_HAsciiString("");
                    _facetedBrep.Init(name, cShell);
                    _done = true;
                    // Kerf: OCCT tests for no tessellated shell and then reads it, so every solid failed; a tessellated
                    // shell, when there is one, becomes the tessellated solid, as the shell's form does (decision 23102)
                    if (stepB.TessellatedValue() is TessellatedShell tessShell)
                    {
                        var tessSolid = new TessellatedSolid();
                        var tessName = new TCollection_HAsciiString("");
                        var items = new NCollection_HArray1<TessellatedStructuredItem>(1, tessShell.NbItems());
                        for (int i = 1; i <= tessShell.NbItems(); ++i)
                        {
                            items.SetValue(i, tessShell.ItemsValue(i));
                        }

                        bool hasGeomLink = _facetedBrep is not null;
                        tessSolid.Init(tessName, items, hasGeomLink, _facetedBrep);
                        _tessellatedItem = tessSolid;
                    }
                }
                else
                {
                    _done = false;
                    var errShape = new ShapeMapper(outerShell);
                    fp.AddWarning(errShape, " Closed Outer Shell from Solid not mapped to FacetedBrep");
                }
            }
            else
            {
                _done = false;
                var errShape = new ShapeMapper(outerShell);
                fp.AddWarning(errShape, " Shell not closed; not mapped to FacetedBrep");
            }
        }
        else
        {
            _done = false;
            var errShape = new ShapeMapper(outerShell);
            fp.AddWarning(errShape, " Solid contains no Outer Shell to be mapped to FacetedBrep");
        }
    }

    public MakeFacetedBrep(in Solid aSolid, FinderProcess fp)
        : this(aSolid, fp, new Factors())
    {
    }

    /// <exception cref="NotDoneException">The mapping wasn't done.</exception>
    public StepFacetedBrep Value()
    {
        StdFail_NotDone_Raise_if(!_done, "TopoDSToStep_MakeFacetedBrep::Value() - no result");
        return _facetedBrep;
    }

    /// <summary>The tessellated item made, the optional result.</summary>
    /// <exception cref="NotDoneException">The mapping wasn't done.</exception>
    public TessellatedItem TessellatedValue()
    {
        StdFail_NotDone_Raise_if(!_done, "TopoDSToStep_MakeFacetedBrep::TessellatedValue() - no result");
        return _tessellatedItem;
    }
}
