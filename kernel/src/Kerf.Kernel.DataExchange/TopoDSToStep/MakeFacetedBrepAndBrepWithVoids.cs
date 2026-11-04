// Port of: src/DataExchange/TKDESTEP/TopoDSToStep/TopoDSToStep_MakeFacetedBrepAndBrepWithVoids.hxx
// Port of: src/DataExchange/TKDESTEP/TopoDSToStep/TopoDSToStep_MakeFacetedBrepAndBrepWithVoids.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings
#pragma warning disable CS0168 // locals OCCT declares and never uses

using Kerf.Engine.Exchange.Framework.Transfer;
using Kerf.Engine.Exchange.Step.Data;
using Kerf.Engine.Exchange.Step.Shapes;
using Kerf.Engine.Exchange.Step.Visual;
using Kerf.Engine.Foundation;
using Kerf.Engine.Topology;
using Kerf.Topology;
using Occt;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.Exchange.Step.Export;

/// <summary>Makes a faceted_brep_and_brep_with_voids from a solid holding more than one closed shell.</summary>
internal sealed class MakeFacetedBrepAndBrepWithVoids : ExportRoot
{
    private StepFacetedBrepAndBrepWithVoids _facetedBrepAndBrepWithVoids;
    private TessellatedItem _tessellatedItem;

    public MakeFacetedBrepAndBrepWithVoids(in Solid aSolid, FinderProcess fp, in Factors localFactors,
                                                        ProgressRange progress = default)
    {
        _done = false;
        Iterator it = new Iterator();
        var map = new Dictionary<Shape, Transient>(ShapeHasher.Instance);
        var s = new NCollection_Sequence<Transient>();
        Shell outShell;

        StepTopologicalRepresentationItem item;
        StepClosedShell outer = null, cShell;
        StepOrientedClosedShell ocShell;
        NCollection_HArray1<StepOrientedClosedShell> voids;
        var tessShells = new NCollection_Sequence<Transient>();

        outShell = global::Kerf.Engine.Classification.BRepClass3d.OuterShell(aSolid);
        StepModel stepModel = fp.Model() as StepModel;
        var stepB = new ExportBuilder();
        var tool = new ExportTool(stepModel);

        if (!outShell.IsNull)
        {
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
                    ProgressRange range = ps.Next();
                    Shell currentShell = ShapeCasts.Shell(it.Value());
                    if (it.Value().IsClosed)
                    {
                        tool.Init(map, false, stepModel.InternalParameters.WriteSurfaceCurMode ? 1 : 0);
                        stepB.Init(currentShell, tool, fp, 0 /* OCCT: false */, localFactors, range);
                        global::Kerf.Engine.Exchange.Step.Export.TopoDSToStep.AddResult(fp, tool);

                        if (stepB.IsDone)
                        {
                            cShell = stepB.Value() as StepClosedShell;
                            if (outShell.IsEqual(it.Value()))
                            {
                                outer = cShell;
                            }
                            else
                            {
                                s.Append(cShell);
                            }

                            TessellatedItem tessShell = stepB.TessellatedValue();
                            if (tessShell is not null)
                            {
                                tessShells.Append(tessShell);
                            }
                        }
                        else
                        {
                            var errShape = new ShapeMapper(currentShell);
                            fp.AddWarning(errShape, " Shell from Solid not mapped to FacetedBrepAndBrepWithVoids");
                        }
                    }
                    else
                    {
                        _done = false;
                        var errShape = new ShapeMapper(currentShell);
                        fp.AddWarning(errShape, " Shell from Solid not closed; not mapped to FacetedBrepAndBrepWithVoids");
                    }
                }
            }

            if (!ps.More())
            {
                return;
            }
        }

        int n = s.Length();
        if (n >= 1)
        {
            voids = new NCollection_HArray1<StepOrientedClosedShell>(1, n);
            var name = new TCollection_HAsciiString("");
            for (int i = 1; i <= n; i++)
            {
                ocShell = new StepOrientedClosedShell();
                ocShell.Init(name, s.Value(i) as StepClosedShell, true);
                voids.SetValue(i, ocShell);
            }

            _facetedBrepAndBrepWithVoids = new StepFacetedBrepAndBrepWithVoids();
            _facetedBrepAndBrepWithVoids.Init(name, outer, voids);

            if (!tessShells.IsEmpty())
            {
                var tessSolid = new TessellatedSolid();
                var tessName = new TCollection_HAsciiString("");
                int nbItems = 0;
                for (int k = 1; k <= tessShells.Length(); k++)
                {
                    TessellatedShell tessShell = tessShells.Value(k) as TessellatedShell;
                    nbItems += tessShell.NbItems();
                }

                var items = new NCollection_HArray1<TessellatedStructuredItem>(1, nbItems);
                for (int k = 1; k <= tessShells.Length(); k++)
                {
                    TessellatedShell tessShell = tessShells.Value(k) as TessellatedShell;
                    for (int i = 1; i <= tessShell.NbItems(); ++i)
                    {
                        items.SetValue(i, tessShell.ItemsValue(i)); // OCCT: each shell's items overwrite the first ones
                    }
                }

                bool hasGeomLink = _facetedBrepAndBrepWithVoids is not null;
                tessSolid.Init(tessName, items, hasGeomLink, _facetedBrepAndBrepWithVoids);
                _tessellatedItem = tessSolid;
            }

            _done = true;
        }
        else
        {
            _done = false;
            var errShape = new ShapeMapper(aSolid);
            fp.AddWarning(errShape, " Solid contains no Shell to be mapped to FacetedBrepAndBrepWithVoids");
        }
    }

    public MakeFacetedBrepAndBrepWithVoids(in Solid aSolid, FinderProcess fp)
        : this(aSolid, fp, new Factors())
    {
    }

    /// <exception cref="NotDoneException">The mapping wasn't done.</exception>
    public StepFacetedBrepAndBrepWithVoids Value()
    {
        StdFail_NotDone_Raise_if(!_done, "TopoDSToStep_MakeFacetedBrepAndBrepWithVoids::Value() - no result");
        return _facetedBrepAndBrepWithVoids;
    }

    /// <summary>The tessellated item made, the optional result.</summary>
    /// <exception cref="NotDoneException">The mapping wasn't done.</exception>
    public TessellatedItem TessellatedValue()
    {
        StdFail_NotDone_Raise_if(!_done, "TopoDSToStep_MakeFacetedBrepAndBrepWithVoids::TessellatedValue() - no result");
        return _tessellatedItem;
    }
}
