// Port of: src/DataExchange/TKDESTEP/TopoDSToStep/TopoDSToStep_MakeGeometricCurveSet.hxx
// Port of: src/DataExchange/TKDESTEP/TopoDSToStep/TopoDSToStep_MakeGeometricCurveSet.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Exchange.Framework.Transfer;
using Kerf.Engine.Exchange.Step.Data;
using Kerf.Engine.Exchange.Step.Shapes;
using Kerf.Engine.Foundation;
using Kerf.Engine.Topology;
using Kerf.Topology;
using Occt;

namespace Kerf.Engine.Exchange.Step.Export;

/// <summary>Makes a geometric_curve_set of the (trimmed) curves of a shape's edges.</summary>
internal sealed class MakeGeometricCurveSet : ExportRoot
{
    private StepGeometricCurveSet _geometricCurveSet;

    public MakeGeometricCurveSet(in Shape aShape, FinderProcess fp, in Factors localFactors)
    {
        _done = false;
        NCollection_HSequence<Transient> itemList;
        var map = new Dictionary<Shape, Transient>(ShapeHasher.Instance);
        StepModel stepModel = fp.Model() as StepModel;
        var tool = new ExportTool(map, false, stepModel.InternalParameters.WriteSurfaceCurMode ? 1 : 0);
        var wirefB = new WireframeBuilder(aShape, tool, localFactors);
        global::Kerf.Engine.Exchange.Step.Export.TopoDSToStep.AddResult(fp, tool);
        var gcSet = new StepGeometricCurveSet();
        var empty = new TCollection_HAsciiString("");

        if (wirefB.IsDone)
        {
            itemList = wirefB.Value();
            int nbItem = itemList.Length();
            if (nbItem > 0)
            {
                var gss = new NCollection_HArray1<StepGeometricSetSelect>(1, nbItem);
                for (int i = 1; i <= nbItem; i++)
                {
                    var select = new StepGeometricSetSelect();
                    select.SetValue(itemList.Value(i));
                    gss.SetValue(i, select); // a new SELECT each time: the copy C++ stores
                }

                gcSet.SetName(empty);
                gcSet.SetElements(gss);
                _geometricCurveSet = gcSet;
                _done = true;
            }
        }
    }

    public MakeGeometricCurveSet(in Shape aShape, FinderProcess fp)
        : this(aShape, fp, new Factors())
    {
    }

    /// <exception cref="NotDoneException">The mapping wasn't done.</exception>
    public StepGeometricCurveSet Value()
    {
        StdFail_NotDone_Raise_if(!_done, "TopoDSToStep_MakeGeometricCurveSet::Value() - no result");
        return _geometricCurveSet;
    }
}
