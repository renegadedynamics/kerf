// Port of: src/ModelingAlgorithms/TKShHealing/ShapeAnalysis/ShapeAnalysis_BoxBndTree.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeAnalysis/ShapeAnalysis_BoxBndTree.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2005-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using static Occt.ShapeExtend_Status;

namespace Occt;

/// <summary>
/// The selector of the tree of wire end boxes used by <see cref="ShapeAnalysis_FreeBounds"/>'s ConnectWiresToWires:
/// accepts the wire whose end is shared with (or closest to) the current wire's ends.
/// </summary>
/// <remarks>
/// A working object for ported code (decision 21). <c>Accept</c> reads the wires of the sequence it is given; it
/// writes nothing into shapes.
/// </remarks>
public sealed class ShapeAnalysis_BoxBndTreeSelector : SpatialTree<int, Box>.Selector
{
    private Box myFBox;
    private Box myLBox;
    private readonly NCollection_HArray1<Shape> mySeq;
    private readonly bool myShared;
    private int myNb;
    private Vertex myFVertex;
    private Vertex myLVertex;
    private Point3D myFPnt;
    private Point3D myLPnt;
    private readonly NCollection_Map<int> myList = new();
    private double myTol;
    private double myMin3d;
    private readonly NCollection_Array1<int> myArrIndices;
    private int myStatus;

    /// <summary>A selector over the wires of <paramref name="theSeq"/>, by shared vertices or by distance.</summary>
    public ShapeAnalysis_BoxBndTreeSelector(NCollection_HArray1<Shape> theSeq, bool theShared)
    {
        mySeq = theSeq;
        myShared = theShared;
        myNb = 0;
        myTol = 1e-7;
        myMin3d = 1e-7;
        myArrIndices = new NCollection_Array1<int>(1, 2);
        myStatus = ShapeExtend.EncodeStatus(ShapeExtend_OK);
        myFBox = new Box();
        myLBox = new Box();
        myFVertex = new Vertex();
        myLVertex = new Vertex();
        myFPnt = new Point3D();
        myLPnt = new Point3D();
        myArrIndices.Init(0);
    }

    /// <summary>Sets the boxes of the current wire's first and last vertices.</summary>
    public void DefineBoxes(in Box theFBox, in Box theLBox)
    {
        myFBox = theFBox;
        myLBox = theLBox;
        myArrIndices.Init(0);
    }

    /// <summary>Sets the current wire's first and last vertices.</summary>
    public void DefineVertexes(Vertex theVf, Vertex theVl)
    {
        myFVertex = theVf;
        myLVertex = theVl;
        myStatus = ShapeExtend.EncodeStatus(ShapeExtend_OK);
    }

    /// <summary>Sets the current wire's first and last points.</summary>
    public void DefinePnt(in Point3D theFPnt, in Point3D theLPnt)
    {
        myFPnt = theFPnt;
        myLPnt = theLPnt;
        myStatus = ShapeExtend.EncodeStatus(ShapeExtend_OK);
    }

    /// <summary>The index of the wire accepted.</summary>
    public int GetNb() => myNb;

    /// <summary>Sets the index of the wire accepted.</summary>
    public void SetNb(int theNb) => myNb = theNb;

    /// <summary>Excludes the wire of the given index from the selection.</summary>
    public void LoadList(int elem) => myList.Add(elem);

    /// <summary>Clears the stop flag.</summary>
    public void SetStop() => _stop = false;

    /// <summary>Sets the tolerance of the distance mode.</summary>
    public void SetTolerance(double theTol)
    {
        myTol = theTol;
        myMin3d = theTol;
        myStatus = ShapeExtend.EncodeStatus(ShapeExtend_OK);
    }

    /// <summary>True if the wire of the given index is excluded.</summary>
    public bool ContWire(int nbWire) => myList.Contains(nbWire);

    /// <summary>
    /// The status of the last acceptance: DONE1 (DONE2) the current wire's last vertex is the wire's first (last);
    /// DONE3 (DONE4) its first vertex is the wire's last (first); FAIL2 none.
    /// </summary>
    public bool LastCheckStatus(ShapeExtend_Status theStatus) => ShapeExtend.DecodeStatus(myStatus, theStatus);

    /// <summary>True if the box is out of both end boxes.</summary>
    public override bool Reject(in Box theBnd)
    {
        bool fch = myFBox.IsOut(theBnd);
        bool lch = myLBox.IsOut(theBnd);
        return fch && lch;
    }

    private const int First = 1;
    private const int Last = 2;

    /// <summary>Accepts the wire of the given index if it connects to the current wire.</summary>
    public override bool Accept(in int theObj)
    {
        if (theObj < 1 || theObj > mySeq.Length())
        {
            throw new NoSuchObjectException("ShapeAnalysis_BoxBndTreeSelector::Accept : no such object for current index");
        }
        bool IsAccept = false;
        if (myList.Contains(theObj))
        {
            return false;
        }

        Wire W = ShapeCasts.Wire(mySeq.Value(theObj));
        ShapeAnalysis.FindBounds(W, out Vertex V1, out Vertex V2);
        if (myShared)
        {
            if (myLVertex.IsSame(V1))
            {
                myStatus = ShapeExtend.EncodeStatus(ShapeExtend_DONE1);
                IsAccept = true;
                myArrIndices[Last] = theObj;
            }
            else
            {
                if (myLVertex.IsSame(V2))
                {
                    myStatus = ShapeExtend.EncodeStatus(ShapeExtend_DONE2);
                    IsAccept = true;
                    myArrIndices[Last] = theObj;
                }
                else
                {
                    if (myFVertex.IsSame(V2))
                    {
                        myStatus = ShapeExtend.EncodeStatus(ShapeExtend_DONE3);
                        IsAccept = true;
                        myArrIndices[First] = theObj;
                    }
                    else
                    {
                        if (myFVertex.IsSame(V1))
                        {
                            myStatus = ShapeExtend.EncodeStatus(ShapeExtend_DONE4);
                            IsAccept = true;
                            myArrIndices[First] = theObj;
                        }
                        else
                        {
                            myStatus = ShapeExtend.EncodeStatus(ShapeExtend_FAIL2);
                        }
                    }
                }
            }

            if (IsAccept)
            {
                SetNb(theObj);
                if (myArrIndices[Last] != 0)
                {
                    _stop = true;
                }
                return true;
            }
            else
            {
                _stop = false;
            }
        }

        else
        {
            Point3D p1 = Tool.Pnt(V1);
            Point3D p2 = Tool.Pnt(V2);

            double tailhead, tailtail, headhead, headtail;
            tailhead = p1.Distance(myLPnt);
            tailtail = p2.Distance(myLPnt);
            headhead = p1.Distance(myFPnt);
            headtail = p2.Distance(myFPnt);
            double dm1 = tailhead, dm2 = headtail;
            int res1 = 0, res2 = 0;
            if (tailhead > tailtail)
            {
                res1 = 1;
                dm1 = tailtail;
            }
            if (headtail > headhead)
            {
                res2 = 1;
                dm2 = headhead;
            }
            int result = res1;
            double min3d;
            min3d = Min(dm1, dm2);
            if (min3d > myMin3d)
            {
                return false;
            }

            int minInd = (dm1 > dm2 ? First : Last);
            int maxInd = (dm1 > dm2 ? Last : First);
            myArrIndices[minInd] = theObj;
            if ((min3d - myMin3d) > RealSmall())
            {
                myArrIndices[maxInd] = 0;
            }

            myMin3d = min3d;
            if (min3d > myTol)
            {
                myStatus = ShapeExtend.EncodeStatus(ShapeExtend_FAIL2);
                return false;
            }

            int anObj = (myArrIndices[Last] != 0 ? myArrIndices[Last] : myArrIndices[First]);
            SetNb(anObj);

            if (min3d == 0 && minInd == Last)
            {
                _stop = true;
            }

            if (dm1 > dm2)
            {
                dm1 = dm2;
                result = res2 + 2;
            }
            if (anObj == theObj)
            {
                switch (result)
                {
                    case 0:
                        myStatus = ShapeExtend.EncodeStatus(ShapeExtend_DONE1);
                        break;
                    case 1:
                        myStatus = ShapeExtend.EncodeStatus(ShapeExtend_DONE2);
                        break;
                    case 2:
                        myStatus = ShapeExtend.EncodeStatus(ShapeExtend_DONE3);
                        break;
                    case 3:
                        myStatus = ShapeExtend.EncodeStatus(ShapeExtend_DONE4);
                        break;
                }
            }
            return true;
        }

        return false;
    }
}
