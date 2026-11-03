// Port of: src/DataExchange/TKDESTEP/TopoDSToStep/TopoDSToStep_MakeGeometricCurveSet.hxx
// Port of: src/DataExchange/TKDESTEP/TopoDSToStep/TopoDSToStep_MakeGeometricCurveSet.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Foundation;

namespace Occt;

/// <summary>Makes a geometric_curve_set of the (trimmed) curves of a shape's edges.</summary>
public sealed class TopoDSToStep_MakeGeometricCurveSet : TopoDSToStep_Root
{
    private StepShape_GeometricCurveSet theGeometricCurveSet;

    public TopoDSToStep_MakeGeometricCurveSet(in TopoDS_Shape aShape, Transfer_FinderProcess FP, in StepData_Factors theLocalFactors)
    {
        done = false;
        NCollection_HSequence<Transient> itemList;
        var aMap = new NCollection_DataMap<TopoDS_Shape, Transient>(TopTools_ShapeMapHasher.Instance);
        StepData_StepModel aStepModel = FP.Model() as StepData_StepModel;
        var aTool = new TopoDSToStep_Tool(aMap, false, aStepModel.InternalParameters.WriteSurfaceCurMode ? 1 : 0);
        var wirefB = new TopoDSToStep_WireframeBuilder(aShape, aTool, theLocalFactors);
        TopoDSToStep.AddResult(FP, aTool);
        var aGCSet = new StepShape_GeometricCurveSet();
        var empty = new TCollection_HAsciiString("");

        if (wirefB.IsDone())
        {
            itemList = wirefB.Value();
            int nbItem = itemList.Length();
            if (nbItem > 0)
            {
                var aGSS = new NCollection_HArray1<StepShape_GeometricSetSelect>(1, nbItem);
                for (int i = 1; i <= nbItem; i++)
                {
                    var select = new StepShape_GeometricSetSelect();
                    select.SetValue(itemList.Value(i));
                    aGSS.SetValue(i, select); // a new SELECT each time: the copy C++ stores
                }

                aGCSet.SetName(empty);
                aGCSet.SetElements(aGSS);
                theGeometricCurveSet = aGCSet;
                done = true;
            }
        }
    }

    public TopoDSToStep_MakeGeometricCurveSet(in TopoDS_Shape aShape, Transfer_FinderProcess FP)
        : this(aShape, FP, new StepData_Factors())
    {
    }

    /// <exception cref="NotDoneException">The mapping wasn't done.</exception>
    public StepShape_GeometricCurveSet Value()
    {
        StdFail_NotDone_Raise_if(!done, "TopoDSToStep_MakeGeometricCurveSet::Value() - no result");
        return theGeometricCurveSet;
    }
}
