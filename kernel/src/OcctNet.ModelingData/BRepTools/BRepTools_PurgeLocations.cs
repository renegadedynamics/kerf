// Port of: src/ModelingData/TKBRep/BRepTools/BRepTools_PurgeLocations.hxx
// Port of: src/ModelingData/TKBRep/BRepTools/BRepTools_PurgeLocations.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2021-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// Removes from a shape the locations that aren't rigid motions (mirrors and scalings), by applying them to the
/// geometry with <see cref="BRepTools_TrsfModification"/>, and rebuilding the shape with <see cref="BRepTools_ReShape"/>.
/// </summary>
/// <remarks>
/// A working object. <see cref="GetResult"/> and <see cref="ModifiedShape"/> are publication points: they freeze the
/// shapes they return (docs/phase3-design.md, section 4.2). The input is never written into.
/// </remarks>
public sealed class BRepTools_PurgeLocations
{
    private bool myDone;
    private TopoDS_Shape myShape = new();
    private readonly NCollection_IndexedMap<TopoDS_Shape> myMapShapes = new(TopTools_ShapeMapHasher.Instance);
    private readonly TopTools_LocationSet myLocations = new();
    private readonly NCollection_DataMap<TopoDS_Shape, TopoDS_Shape> myMapNewShapes = new(TopTools_ShapeMapHasher.Instance);
    private BRepTools_ReShape? myReShape;

    /// <summary>A purge, not yet performed.</summary>
    public BRepTools_PurgeLocations()
    {
        myDone = false;
    }

    /// <summary>Removes the locations that aren't rigid motions from the shape. True if every one could be applied.</summary>
    public bool Perform(in TopoDS_Shape theShape)
    {
        myShape = theShape;
        myMapShapes.Clear();
        myLocations.Clear();
        myDone = true;
        AddShape(myShape);

        // Check locations;
        int ind;
        var aBadTrsfInds = new NCollection_DynamicArray<int>();
        for (ind = 1; ; ++ind)
        {
            TopLoc_Location aLoc = myLocations.Location(ind);

            if (aLoc.IsIdentity())
            {
                break;
            }

            gp_Trsf aTrsf = aLoc.Transformation();
            bool isBadTrsf =
                aTrsf.IsNegative()
                || (Math.Abs(Math.Abs(aTrsf.ScaleFactor()) - 1.0) > TopLoc_Location.ScalePrec());
            if (isBadTrsf)
            {
                aBadTrsfInds.Append(ind);
            }
        }

        if (aBadTrsfInds.IsEmpty())
        {
            return myDone;
        }

        int aNbShapes = myMapShapes.Extent();
        myMapNewShapes.Clear();
        int inds;
        for (inds = 1; inds <= aNbShapes; ++inds)
        {
            TopoDS_Shape anS = myMapShapes[inds];
            int aLocInd = myLocations.Index(anS.Location());
            if (aLocInd == 0)
            {
                continue;
            }
            int il;
            for (il = 0; il < aBadTrsfInds.Length(); ++il)
            {
                if (aBadTrsfInds[il] == aLocInd)
                {
                    bool isDone = PurgeLocation(anS, out TopoDS_Shape aTrS);
                    myDone = myDone && isDone;
                    myMapNewShapes.Bind(anS, aTrS);
                    break;
                }
            }
        }

        if (myReShape is null)
        {
            myReShape = new BRepTools_ReShape();
        }
        else
        {
            myReShape.Clear();
        }
        // OCCT.NET: the map's order isn't OCCT's (decision 18); the replacements are of distinct shapes, so the
        // order doesn't change the result.
        foreach (KeyValuePair<TopoDS_Shape, TopoDS_Shape> anIter in myMapNewShapes)
        {
            TopoDS_Shape anOldS = anIter.Key;
            TopoDS_Shape aNewS = anIter.Value;
            myReShape.Replace(anOldS, aNewS);
        }

        myShape = myReShape.InternalApply(myShape);

        return myDone;
    }

    private bool PurgeLocation(in TopoDS_Shape theS, out TopoDS_Shape theRes)
    {
        bool isDone = true;
        TopLoc_Location aRefLoc = theS.Location();
        bool isEmpty = aRefLoc.IsIdentity();
        if (isEmpty)
        {
            theRes = theS;
            return isDone;
        }

        TopLoc_Location aNullLoc = new TopLoc_Location();
        theRes = theS.Located(aNullLoc);

        while (!isEmpty)
        {
            TopLoc_Datum3D aFD = aRefLoc.FirstDatum();
            gp_Trsf aTrsf = aFD.Trsf();
            int aFP = aRefLoc.FirstPower();
            bool isBad = aTrsf.IsNegative()
                         || (Math.Abs(Math.Abs(aTrsf.ScaleFactor()) - 1.0) > TopLoc_Location.ScalePrec());
            TopLoc_Location aLoc = new TopLoc_Location(aFD);
            aLoc = aLoc.Powered(aFP);
            aTrsf = aLoc.Transformation();
            if (isBad)
            {
                BRepTools_TrsfModification aModification = new BRepTools_TrsfModification(aTrsf);
                BRepTools_Modifier aModifier = new BRepTools_Modifier(theRes, aModification);
                if (aModifier.IsDone())
                {
                    theRes = aModifier.InternalModifiedShape(theRes);
                }
                else
                {
                    isDone = false;
                    theRes = theRes.Moved(aLoc);
                }
            }
            else
            {
                theRes = theRes.Moved(aLoc);
            }

            aRefLoc = aRefLoc.NextLocation();
            isEmpty = aRefLoc.IsIdentity();
        }

        return isDone;
    }

    private void AddShape(in TopoDS_Shape theS)
    {
        myMapShapes.Add(theS);
        myLocations.Add(theS.Location());

        var It = new TopoDS_Iterator(theS, false, false);
        while (It.More())
        {
            AddShape(It.Value());
            It.Next();
        }
    }

    /// <summary>The shape without the locations that aren't rigid motions, frozen.</summary>
    public TopoDS_Shape GetResult()
    {
        myShape.TShape()?.FreezeGraph();
        return myShape;
    }

    /// <summary>True if every location could be applied.</summary>
    public bool IsDone() => myDone;

    /// <summary>The shape that a sub-shape of the input became (the sub-shape itself if unchanged), frozen.</summary>
    public TopoDS_Shape ModifiedShape(in TopoDS_Shape theInitShape)
    {
        TopoDS_Shape aShape = theInitShape;
        if (myMapNewShapes.IsBound(theInitShape))
        {
            aShape = myMapNewShapes.Find(theInitShape);
        }
        aShape.TShape()?.FreezeGraph();
        return aShape;
    }
}
