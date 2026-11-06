// Port of: src/DataExchange/TKDESTEP/StepToTopoDS/StepToTopoDS_TranslateSolid.hxx
// Port of: src/DataExchange/TKDESTEP/StepToTopoDS/StepToTopoDS_TranslateSolid.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2022-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Exchange.Framework.Transfer;
using Kerf.Engine.Exchange.Step.Data;
using Kerf.Engine.Exchange.Step.Visual;
using Kerf.Engine.Foundation;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Topology;

namespace Kerf.Engine.Exchange.Step.Import;

/// <summary>Translates a tessellated_solid into a TopoDS_Solid of faces carrying triangulations.</summary>
/// <remarks><see cref="Value"/> freezes the result; the translators use <c>InternalValue</c> (decision 1247).</remarks>
internal sealed class TranslateSolid : ImportRoot
{
    private TranslateSolidError _error;
    private Shape _result = new Shape();

    public TranslateSolid()
    {
        _error = TranslateSolidError.TranslateSolidOther;
        _done = false;
    }

    public void Init(TessellatedSolid tSo, TransientProcess tp, ImportTool tool,
                     NMTool nmTool, bool readTessellatedWhenNoBRepOnly, ref bool hasGeom,
                     in Factors localFactors, ProgressRange progress = default)
    {
        if (tSo is null)
        {
            return;
        }

        var b = new BrepBuilder();
        Shell sh = new Shell();
        Solid so = new Solid();

        int nb = tSo.NbItems();
        using var ps = new ProgressScope(progress, "Face", nb);

        if (tSo.HasGeometricLink && tp.IsBound(tSo.GeometricLink()))
        {
            ShapeBinder binder = tp.Find(tSo.GeometricLink()) as ShapeBinder;
            if (binder is not null)
            {
                so = binder.Solid();
            }
        }

        bool newSolid = false;
        if (so.IsNull)
        {
            b.MakeShell(ref sh);
            b.MakeSolid(ref so);
            newSolid = true;
            hasGeom = false;
        }

        TransientProcess aTP = tool.TransientProcess();

        var tranTF = new TranslateFace();
        tranTF.SetPrecision(Precision());
        tranTF.SetMaxTol(MaxTol());

        for (int i = 1; i <= nb && ps.More(); i++, ps.Next())
        {
            TessellatedStructuredItem item = tSo.ItemsValue(i);
            if (item is TessellatedFace)
            {
                TessellatedFace tFace = item as TessellatedFace;
                bool hasFaceGeom = false;
                tranTF.Init(tFace, tool, nmTool, readTessellatedWhenNoBRepOnly, ref hasFaceGeom, localFactors);
                if (tranTF.IsDone)
                {
                    if (newSolid)
                    {
                        b.Add(sh, ShapeCasts.Face(tranTF.InternalValue()));
                    }

                    hasGeom &= hasFaceGeom;
                }
                else
                {
                    aTP.AddWarning(item, " Tessellated face if not mapped to TopoDS");
                }
            }
            else
            {
                aTP.AddWarning(item, " Face is not of TessellatedFace Type; not mapped to TopoDS");
            }
        }

        if (newSolid)
        {
            b.Add(so, sh);
        }

        _result = so;
        _error = TranslateSolidError.TranslateSolidDone;
        _done = true;
    }

    public void Init(TessellatedSolid tSo, TransientProcess tp, ImportTool tool,
                     NMTool nmTool, bool readTessellatedWhenNoBRepOnly, ref bool hasGeom)
        => Init(tSo, tp, tool, nmTool, readTessellatedWhenNoBRepOnly, ref hasGeom, new Factors());

    /// <summary>The solid, frozen.</summary>
    /// <exception cref="NotDoneException">The translation wasn't done.</exception>
    public Shape Value()
    {
        Shape shape = InternalValue();
        shape.TShape()?.FreezeGraph();
        return shape;
    }

    /// <summary>As <see cref="Value"/>, without freezing: the reader's working shape.</summary>
    internal Shape InternalValue()
    {
        StdFail_NotDone_Raise_if(!_done, "StepToTopoDS_TranslateSolid::Value() - no result");
        return _result;
    }

    public TranslateSolidError Error() => _error;
}
