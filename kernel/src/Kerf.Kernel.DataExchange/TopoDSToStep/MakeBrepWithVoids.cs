// Port of: src/DataExchange/TKDESTEP/TopoDSToStep/TopoDSToStep_MakeBrepWithVoids.hxx
// Port of: src/DataExchange/TKDESTEP/TopoDSToStep/TopoDSToStep_MakeBrepWithVoids.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings
#pragma warning disable CS0168 // locals OCCT declares and never uses

using Kerf.Engine.Exchange.Framework.Transfer;
using Kerf.Engine.Exchange.Step.Data;
using Kerf.Engine.Exchange.Step.Shapes;
using Kerf.Engine.Exchange.Step.Visual;
using Kerf.Engine.Foundation;
using Kerf.Engine.Topology;
using Kerf.Topology;
using Kerf.Engine;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.Exchange.Step.Export;

/// <summary>Makes a brep_with_voids from a solid holding more than one closed shell.</summary>
internal sealed class MakeBrepWithVoids : ExportRoot
{
    private StepBrepWithVoids _brepWithVoids;
    private TessellatedItem _tessellatedItem;

    public MakeBrepWithVoids(in Solid aSolid, FinderProcess fp, in Factors localFactors,
                                          ProgressRange progress = default)
    {
        _done = false;
        Iterator it = new Iterator();
        var map = new Dictionary<Shape, Transient>(ShapeHasher.Instance);
        var s = new NCollection_Sequence<Transient>();
        Shell outShell = new Shell();

        StepTopologicalRepresentationItem item;
        StepClosedShell outer = null, cShell;
        StepOrientedClosedShell ocShell;
        NCollection_HArray1<StepOrientedClosedShell> voids;
        var tessShells = new NCollection_Sequence<Transient>();

        StepModel stepModel = fp.Model() as StepModel;
        int writeTessGeom = (int)stepModel.InternalParameters.WriteTessellated;

        try
        {
            // BRepClass3d::OuterShell requires geometry when the solid has voids
            outShell = global::Kerf.Engine.Classification.BRepClass3d.OuterShell(aSolid);
        }
        catch (Exception exception) when (exception is not OutOfMemoryException and not OperationCanceledException)
        {
            // OCCT: catch (...)
        }

        var stepB = new ExportBuilder();
        var tool = new ExportTool(stepModel);

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
                Shell currentShell = ShapeCasts.Shell(it.Value());
                if (!outShell.IsNull && !outShell.IsEqual(currentShell))
                { //: e0 abv 25 Mar 98: voids should be reversed
                    //: according to EXPRESS for ABSR
                    currentShell.Reverse();
                }

                //: d7 abv 16 Mar 98: try to treat 'open' shells as closed since flag
                // IsClosed() is often incorrect (taken from MakeManifoldSolid(Solid))
                tool.Init(map, false, stepModel.InternalParameters.WriteSurfaceCurMode ? 1 : 0);
                stepB.Init(currentShell, tool, fp, writeTessGeom, localFactors, ps.Next());
                global::Kerf.Engine.Exchange.Step.Export.TopoDSToStep.AddResult(fp, tool);

                if (stepB.IsDone)
                {
                    cShell = stepB.Value() as StepClosedShell;
                    // si OPEN on le force a CLOSED mais que c est une honte !
                    if (cShell is null)
                    {
                        StepOpenShell oShell = stepB.Value() as StepOpenShell;
                        if (oShell is not null)
                        {
                            cShell = new StepClosedShell();
                            cShell.Init(oShell.Name(), oShell.CfsFaces());
                        }
                    }

                    TessellatedItem tessShell = stepB.TessellatedValue();
                    if (tessShell is not null)
                    {
                        tessShells.Append(tessShell);
                    }

                    if (!outShell.IsNull && cShell is not null)
                    {
                        if (outShell.IsEqual(it.Value()))
                        {
                            outer = cShell;
                        }
                        else
                        {
                            s.Append(cShell);
                        }
                    }
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
            var name = new TCollection_HAsciiString("");
            voids = new NCollection_HArray1<StepOrientedClosedShell>(1, n);
            for (int i = 1; i <= n; i++)
            {
                ocShell = new StepOrientedClosedShell();
                // Warning : the Oriented Shell Orientation is not always
                //           TRUE.
                //           Shall check the TopoDS_Shell orientation.
                // => if the Shell is reversed, shall create an OrientedShell.
                ocShell.Init(name, s.Value(i) as StepClosedShell, false); //: e0
                voids.SetValue(i, ocShell);
            }

            _brepWithVoids = new StepBrepWithVoids();
            _brepWithVoids.Init(name, outer, voids);
            _done = true;
        }

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
            int j = 1;
            for (int k = 1; k <= tessShells.Length(); k++)
            {
                TessellatedShell tessShell = tessShells.Value(k) as TessellatedShell;
                for (int i = 1; i <= tessShell.NbItems(); ++i)
                {
                    items.SetValue(j++, tessShell.ItemsValue(i));
                }
            }

            bool hasGeomLink = _brepWithVoids is not null;
            tessSolid.Init(tessName, items, hasGeomLink, _brepWithVoids);
            _tessellatedItem = tessSolid;
            _done = true;
        }
    }

    public MakeBrepWithVoids(in Solid aSolid, FinderProcess fp)
        : this(aSolid, fp, new Factors())
    {
    }

    /// <exception cref="NotDoneException">The mapping wasn't done.</exception>
    public StepBrepWithVoids Value()
    {
        StdFail_NotDone_Raise_if(!_done, "TopoDSToStep_MakeBrepWithVoids::Value() - no result");
        return _brepWithVoids;
    }

    /// <summary>The tessellated item made, the optional result.</summary>
    /// <exception cref="NotDoneException">The mapping wasn't done.</exception>
    public TessellatedItem TessellatedValue()
    {
        StdFail_NotDone_Raise_if(!_done, "TopoDSToStep_MakeBrepWithVoids::TessellatedValue() - no result");
        return _tessellatedItem;
    }
}
