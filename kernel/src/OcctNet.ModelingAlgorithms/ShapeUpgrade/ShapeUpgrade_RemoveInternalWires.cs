// Port of: src/ModelingAlgorithms/TKShHealing/ShapeUpgrade/ShapeUpgrade_RemoveInternalWires.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeUpgrade/ShapeUpgrade_RemoveInternalWires.lxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeUpgrade/ShapeUpgrade_RemoveInternalWires.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.ShapeExtend_Status;
using static Occt.TopAbs_ShapeEnum;

namespace Occt;

/// <summary>
/// Removes the internal wires (holes) of faces whose area is less than the minimal area, and, in the remove-face mode
/// (the default), the faces whose outer wire is made only of edges of removed wires.
/// </summary>
/// <remarks>
/// A working object (decision 21). OCCT's modifiable references (<see cref="MinArea"/>, <see cref="RemoveFaceMode"/>)
/// return <c>ref</c>. It only records removals in its context and rebuilds the shape with it, so the shape given is
/// never modified (decision 999). <see cref="GetResult"/> freezes what it returns; ported callers read
/// <see cref="InternalGetResult"/>.
/// </remarks>
public class ShapeUpgrade_RemoveInternalWires : ShapeUpgrade_Tool
{
    /// <summary>The status.</summary>
    protected int myStatus;

    private TopoDS_Shape myShape;
    private TopoDS_Shape myResult;
    private double myMinArea;
    private bool myRemoveFacesMode;
    private readonly NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> myEdgeFaces =
        new(TopTools_ShapeMapHasher.Instance);
    private readonly NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> myRemoveEdges =
        new(TopTools_ShapeMapHasher.Instance);
    private readonly NCollection_Sequence<TopoDS_Shape> myRemovedFaces = new();
    private readonly NCollection_Sequence<TopoDS_Shape> myRemoveWires = new();

    /// <summary>Empty constructor.</summary>
    public ShapeUpgrade_RemoveInternalWires()
    {
        myShape = new TopoDS_Shape();
        myResult = new TopoDS_Shape();
        myMinArea = 0.0;
        myRemoveFacesMode = true;
        myStatus = ShapeExtend.EncodeStatus(ShapeExtend_OK);
        var aContext = new ShapeBuild_ReShape();
        SetContext(aContext);
    }

    /// <summary>Initializes by a shape.</summary>
    public ShapeUpgrade_RemoveInternalWires(in TopoDS_Shape theShape)
    {
        myShape = new TopoDS_Shape();
        myResult = new TopoDS_Shape();
        var aContext = new ShapeBuild_ReShape();
        SetContext(aContext);
        Init(theShape);
    }

    /// <summary>Initializes by a shape.</summary>
    public void Init(in TopoDS_Shape theShape)
    {
        myShape = theShape;
        Context()!.InternalApply(theShape);
        TopExp.MapShapesAndAncestors(myShape, TopAbs_EDGE, TopAbs_FACE, myEdgeFaces);
        myStatus = ShapeExtend.EncodeStatus(ShapeExtend_OK);
        myMinArea = 0.0;
        myRemoveFacesMode = true;
    }

    /// <summary>Removes all the internal wires with an area less than the minimal area; true if something was done.</summary>
    public bool Perform()
    {
        Clear();
        if (myShape.IsNull())
        {
            myStatus |= ShapeExtend.EncodeStatus(ShapeExtend_FAIL1);
            return false;
        }
        var aExpF = new TopExp_Explorer(myShape, TopAbs_FACE);
        for (; aExpF.More(); aExpF.Next())
        {
            TopoDS_Face aF = TopoDS.Face(aExpF.Current());
            removeSmallWire(aF, new TopoDS_Wire());
        }
        if (myRemoveFacesMode)
        {
            removeSmallFaces();
        }

        myResult = Context()!.InternalApply(myShape);
        return Status(ShapeExtend_DONE);
    }

    /// <summary>
    /// Removes the internal wires given (wires), or those of the faces given (faces), with an area less than the
    /// minimal area; true if something was done.
    /// </summary>
    public bool Perform(NCollection_Sequence<TopoDS_Shape> theSeqShapes)
    {
        if (myShape.IsNull())
        {
            myStatus |= ShapeExtend.EncodeStatus(ShapeExtend_FAIL1);
            return false;
        }
        Clear();
        var aWireFaces = new NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>>(TopTools_ShapeMapHasher.Instance);
        int i = 1, nb = theSeqShapes.Length();
        for (; i <= nb; i++)
        {
            TopoDS_Shape aS = theSeqShapes.Value(i);
            if (aS.ShapeType() == TopAbs_FACE)
            {
                removeSmallWire(aS, new TopoDS_Wire());
            }
            else if (aS.ShapeType() == TopAbs_WIRE)
            {
                if (aWireFaces.Extent() == 0)
                {
                    TopExp.MapShapesAndAncestors(myShape, TopAbs_WIRE, TopAbs_FACE, aWireFaces);
                }
                if (aWireFaces.Contains(aS))
                {
                    NCollection_List<TopoDS_Shape> alfaces = aWireFaces.FindFromKey(aS);
                    foreach (TopoDS_Shape aFace in alfaces)
                    {
                        removeSmallWire(aFace, aS);
                    }
                }
            }
        }
        if (myRemoveFacesMode)
        {
            removeSmallFaces();
        }
        myResult = Context()!.InternalApply(myShape);
        return Status(ShapeExtend_DONE);
    }

    /// <summary>Gets the resulting shape, frozen.</summary>
    public TopoDS_Shape GetResult()
    {
        myResult.TShape()?.FreezeGraph();
        return myResult;
    }

    /// <summary><see cref="GetResult"/> without freezing it, for ported callers.</summary>
    internal TopoDS_Shape InternalGetResult() => myResult;

    /// <summary>The minimal area of the holes kept (all holes of smaller area are removed).</summary>
    public ref double MinArea() => ref myMinArea;

    /// <summary>
    /// The mode for removing the faces whose outer wires consist only of edges of removed internal wires (true by
    /// default).
    /// </summary>
    public ref bool RemoveFaceMode() => ref myRemoveFacesMode;

    /// <summary>Returns the removed faces (the tool's own sequence).</summary>
    public NCollection_Sequence<TopoDS_Shape> RemovedFaces() => myRemovedFaces;

    /// <summary>Returns the removed wires (the tool's own sequence).</summary>
    public NCollection_Sequence<TopoDS_Shape> RemovedWires() => myRemoveWires;

    /// <summary>
    /// Queries the status of the last Perform: OK (nothing done), DONE1 (internal wires removed), DONE2 (small faces
    /// removed), FAIL1 (no initial shape), FAIL2 (a sub-shape given isn't in the initial shape).
    /// </summary>
    public bool Status(ShapeExtend_Status theStatus) => ShapeExtend.DecodeStatus(myStatus, theStatus);

    /// <summary>Clears the sequences and the temporary map.</summary>
    protected void Clear()
    {
        myRemoveEdges.Clear();
        myRemovedFaces.Clear();
        myRemoveWires.Clear();
        myStatus = ShapeExtend.EncodeStatus(ShapeExtend_OK);
    }

    // Removes the internal wires with a contour area less than the minimal area.
    private void removeSmallWire(in TopoDS_Shape theFace, in TopoDS_Shape theWire)
    {
        TopoDS_Face aF = TopoDS.Face(theFace);
        TopoDS_Wire anOutW = ShapeAnalysis.OuterWire(aF);
        var aIt = new TopoDS_Iterator(aF);
        for (; aIt.More(); aIt.Next())
        {
            if (aIt.Value().ShapeType() != TopAbs_WIRE || aIt.Value().IsSame(anOutW))
            {
                continue;
            }
            // occ::handle<ShapeExtend_WireData> asewd = new  ShapeExtend_WireData();
            TopoDS_Wire aW = TopoDS.Wire(aIt.Value());
            if (!theWire.IsNull() && !theWire.IsSame(aW))
            {
                continue;
            }
            double anArea = ShapeAnalysis.ContourArea(aW);
            if (anArea < myMinArea - Occt.Precision.Confusion())
            {
                Context()!.Remove(aW);
                myRemoveWires.Append(aW);
                myStatus |= ShapeExtend.EncodeStatus(ShapeExtend_DONE1);
                if (!myRemoveFacesMode)
                {
                    continue;
                }

                var aIte = new TopoDS_Iterator(aW, false);
                for (; aIte.More(); aIte.Next())
                {
                    TopoDS_Shape aE = aIte.Value();
                    if (myRemoveEdges.IsBound(aE))
                    {
                        myRemoveEdges.ChangeFind(aE).Append(aF);
                    }
                    else
                    {
                        var alfaces = new NCollection_List<TopoDS_Shape>();
                        alfaces.Append(aF);
                        myRemoveEdges.Bind(aE, alfaces);
                    }
                }
            }
        }
    }

    // Removes the faces whose outer wire consists of edges of removed (small internal) wires.
    private void removeSmallFaces()
    {
        int i = 1;
        for (; i <= myRemoveWires.Length(); i++)
        {
            TopoDS_Shape aWire = myRemoveWires.Value(i);
            var aIte = new TopoDS_Iterator(aWire, false);
            var aFaceCandidates = new NCollection_IndexedMap<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
            // collecting all faces containing edges from removed wire
            for (; aIte.More(); aIte.Next())
            {
                TopoDS_Shape aEdge = aIte.Value();
                if (!myEdgeFaces.Contains(aEdge))
                {
                    myStatus |= ShapeExtend.EncodeStatus(ShapeExtend_FAIL2);
                    continue;
                }
                NCollection_List<TopoDS_Shape> aLface1 = myEdgeFaces.FindFromKey(aEdge);
                NCollection_List<TopoDS_Shape> aLface2 = myRemoveEdges.Find(aEdge);
                // OCCT's second iterator is not reset for each face of the first list
                var aliter2 = new NCollection_List<TopoDS_Shape>.Iterator(aLface2);
                foreach (TopoDS_Shape aFace1 in aLface1)
                {
                    TopoDS_Shape aF = Context()!.InternalApply(aFace1);
                    if (aF.IsNull())
                    {
                        continue;
                    }
                    bool isFind = false;
                    for (; aliter2.More() && !isFind; aliter2.Next())
                    {
                        TopoDS_Shape aF2 = Context()!.InternalApply(aliter2.Value());
                        isFind = aF.IsSame(aF2);
                    }

                    if (!isFind)
                    {
                        TopoDS_Wire aWout = ShapeAnalysis.OuterWire(TopoDS.Face(aF));
                        bool isOuter = false;
                        var aIter = new TopoDS_Iterator(aWout, false);
                        for (; aIter.More() && !isOuter; aIter.Next())
                        {
                            isOuter = aEdge.IsSame(aIter.Value());
                        }
                        if (isOuter)
                        {
                            aFaceCandidates.Add(aF);
                        }
                    }
                }
            }

            // remove faces which have outer wire consist of only
            // edges from removed wires and
            // seam edges for faces based on conic surface or
            // in the case of a few faces based on the same conic surface
            // the edges belogining these faces.
            int k = 1;
            for (; k <= aFaceCandidates.Extent(); k++)
            {
                TopoDS_Shape aF = aFaceCandidates.FindKey(k);
                TopoDS_Wire anOutW = ShapeAnalysis.OuterWire(TopoDS.Face(aF));
                var asewd = new ShapeExtend_WireData(anOutW);
                int n = 1, nbE = asewd.NbEdges();
                int nbNotRemoved = 0;
                for (; n <= nbE; n++)
                {
                    if (asewd.IsSeam(n))
                    {
                        continue;
                    }
                    TopoDS_Edge aE = asewd.Edge(n);
                    if (!myRemoveEdges.IsBound(aE))
                    {
                        NCollection_List<TopoDS_Shape> aLface3 = myEdgeFaces.FindFromKey(aE);
                        foreach (TopoDS_Shape aFace3 in aLface3)
                        {
                            TopoDS_Shape aF2 = Context()!.InternalApply(aFace3);
                            if (aF2.IsNull())
                            {
                                continue;
                            }
                            if (!aF.IsSame(aF2) && !aFaceCandidates.Contains(aF2))
                            {
                                nbNotRemoved++;
                            }
                        }
                    }
                }

                if (nbNotRemoved == 0)
                {
                    Context()!.Remove(aF);
                    myRemovedFaces.Append(aF);
                }
            }
        }

        if (myRemovedFaces.Length() != 0)
        {
            myStatus |= ShapeExtend.EncodeStatus(ShapeExtend_DONE2);
        }
    }
}
