// Port of: src/ModelingAlgorithms/TKShHealing/ShapeUpgrade/ShapeUpgrade_RemoveInternalWires.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeUpgrade/ShapeUpgrade_RemoveInternalWires.lxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeUpgrade/ShapeUpgrade_RemoveInternalWires.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Topology;
using Kerf.Topology;
using static Occt.ShapeExtend_Status;
using static Kerf.Topology.ShapeType;

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

    private Shape myShape;
    private Shape myResult;
    private double myMinArea;
    private bool myRemoveFacesMode;
    private readonly NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> myEdgeFaces =
        new(ShapeHasher.Instance);
    private readonly NCollection_DataMap<Shape, NCollection_List<Shape>> myRemoveEdges =
        new(ShapeHasher.Instance);
    private readonly NCollection_Sequence<Shape> myRemovedFaces = new();
    private readonly NCollection_Sequence<Shape> myRemoveWires = new();

    /// <summary>Empty constructor.</summary>
    public ShapeUpgrade_RemoveInternalWires()
    {
        myShape = new Shape();
        myResult = new Shape();
        myMinArea = 0.0;
        myRemoveFacesMode = true;
        myStatus = ShapeExtend.EncodeStatus(ShapeExtend_OK);
        var aContext = new ShapeBuild_ReShape();
        SetContext(aContext);
    }

    /// <summary>Initializes by a shape.</summary>
    public ShapeUpgrade_RemoveInternalWires(in Shape theShape)
    {
        myShape = new Shape();
        myResult = new Shape();
        var aContext = new ShapeBuild_ReShape();
        SetContext(aContext);
        Init(theShape);
    }

    /// <summary>Initializes by a shape.</summary>
    public void Init(in Shape theShape)
    {
        myShape = theShape;
        Context()!.InternalApply(theShape);
        Exploration.MapShapesAndAncestors(myShape, ShapeType.Edge, ShapeType.Face, myEdgeFaces);
        myStatus = ShapeExtend.EncodeStatus(ShapeExtend_OK);
        myMinArea = 0.0;
        myRemoveFacesMode = true;
    }

    /// <summary>Removes all the internal wires with an area less than the minimal area; true if something was done.</summary>
    public bool Perform()
    {
        Clear();
        if (myShape.IsNull)
        {
            myStatus |= ShapeExtend.EncodeStatus(ShapeExtend_FAIL1);
            return false;
        }
        var aExpF = new Explorer(myShape, ShapeType.Face);
        for (; aExpF.More(); aExpF.Next())
        {
            Face aF = ShapeCasts.Face(aExpF.Current());
            removeSmallWire(aF, new Wire());
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
    public bool Perform(NCollection_Sequence<Shape> theSeqShapes)
    {
        if (myShape.IsNull)
        {
            myStatus |= ShapeExtend.EncodeStatus(ShapeExtend_FAIL1);
            return false;
        }
        Clear();
        var aWireFaces = new NCollection_IndexedDataMap<Shape, NCollection_List<Shape>>(ShapeHasher.Instance);
        int i = 1, nb = theSeqShapes.Length();
        for (; i <= nb; i++)
        {
            Shape aS = theSeqShapes.Value(i);
            if (aS.ShapeType() == ShapeType.Face)
            {
                removeSmallWire(aS, new Wire());
            }
            else if (aS.ShapeType() == ShapeType.Wire)
            {
                if (aWireFaces.Extent() == 0)
                {
                    Exploration.MapShapesAndAncestors(myShape, ShapeType.Wire, ShapeType.Face, aWireFaces);
                }
                if (aWireFaces.Contains(aS))
                {
                    NCollection_List<Shape> alfaces = aWireFaces.FindFromKey(aS);
                    foreach (Shape aFace in alfaces)
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
    public Shape GetResult()
    {
        myResult.TShape()?.FreezeGraph();
        return myResult;
    }

    /// <summary><see cref="GetResult"/> without freezing it, for ported callers.</summary>
    internal Shape InternalGetResult() => myResult;

    /// <summary>The minimal area of the holes kept (all holes of smaller area are removed).</summary>
    public ref double MinArea() => ref myMinArea;

    /// <summary>
    /// The mode for removing the faces whose outer wires consist only of edges of removed internal wires (true by
    /// default).
    /// </summary>
    public ref bool RemoveFaceMode() => ref myRemoveFacesMode;

    /// <summary>Returns the removed faces (the tool's own sequence).</summary>
    public NCollection_Sequence<Shape> RemovedFaces() => myRemovedFaces;

    /// <summary>Returns the removed wires (the tool's own sequence).</summary>
    public NCollection_Sequence<Shape> RemovedWires() => myRemoveWires;

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
    private void removeSmallWire(in Shape theFace, in Shape theWire)
    {
        Face aF = ShapeCasts.Face(theFace);
        Wire anOutW = ShapeAnalysis.OuterWire(aF);
        var aIt = new Iterator(aF);
        for (; aIt.More(); aIt.Next())
        {
            if (aIt.Value().ShapeType() != ShapeType.Wire || aIt.Value().IsSame(anOutW))
            {
                continue;
            }
            // occ::handle<ShapeExtend_WireData> asewd = new  ShapeExtend_WireData();
            Wire aW = ShapeCasts.Wire(aIt.Value());
            if (!theWire.IsNull && !theWire.IsSame(aW))
            {
                continue;
            }
            double anArea = ShapeAnalysis.ContourArea(aW);
            if (anArea < myMinArea - Kerf.Engine.Foundation.Precision.Confusion())
            {
                Context()!.Remove(aW);
                myRemoveWires.Append(aW);
                myStatus |= ShapeExtend.EncodeStatus(ShapeExtend_DONE1);
                if (!myRemoveFacesMode)
                {
                    continue;
                }

                var aIte = new Iterator(aW, false);
                for (; aIte.More(); aIte.Next())
                {
                    Shape aE = aIte.Value();
                    if (myRemoveEdges.IsBound(aE))
                    {
                        myRemoveEdges.ChangeFind(aE).Append(aF);
                    }
                    else
                    {
                        var alfaces = new NCollection_List<Shape>();
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
            Shape aWire = myRemoveWires.Value(i);
            var aIte = new Iterator(aWire, false);
            var aFaceCandidates = new NCollection_IndexedMap<Shape>(ShapeHasher.Instance);
            // collecting all faces containing edges from removed wire
            for (; aIte.More(); aIte.Next())
            {
                Shape aEdge = aIte.Value();
                if (!myEdgeFaces.Contains(aEdge))
                {
                    myStatus |= ShapeExtend.EncodeStatus(ShapeExtend_FAIL2);
                    continue;
                }
                NCollection_List<Shape> aLface1 = myEdgeFaces.FindFromKey(aEdge);
                NCollection_List<Shape> aLface2 = myRemoveEdges.Find(aEdge);
                // OCCT's second iterator is not reset for each face of the first list
                var aliter2 = new NCollection_List<Shape>.Iterator(aLface2);
                foreach (Shape aFace1 in aLface1)
                {
                    Shape aF = Context()!.InternalApply(aFace1);
                    if (aF.IsNull)
                    {
                        continue;
                    }
                    bool isFind = false;
                    for (; aliter2.More() && !isFind; aliter2.Next())
                    {
                        Shape aF2 = Context()!.InternalApply(aliter2.Value());
                        isFind = aF.IsSame(aF2);
                    }

                    if (!isFind)
                    {
                        Wire aWout = ShapeAnalysis.OuterWire(ShapeCasts.Face(aF));
                        bool isOuter = false;
                        var aIter = new Iterator(aWout, false);
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
                Shape aF = aFaceCandidates.FindKey(k);
                Wire anOutW = ShapeAnalysis.OuterWire(ShapeCasts.Face(aF));
                var asewd = new ShapeExtend_WireData(anOutW);
                int n = 1, nbE = asewd.NbEdges();
                int nbNotRemoved = 0;
                for (; n <= nbE; n++)
                {
                    if (asewd.IsSeam(n))
                    {
                        continue;
                    }
                    Edge aE = asewd.Edge(n);
                    if (!myRemoveEdges.IsBound(aE))
                    {
                        NCollection_List<Shape> aLface3 = myEdgeFaces.FindFromKey(aE);
                        foreach (Shape aFace3 in aLface3)
                        {
                            Shape aF2 = Context()!.InternalApply(aFace3);
                            if (aF2.IsNull)
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
