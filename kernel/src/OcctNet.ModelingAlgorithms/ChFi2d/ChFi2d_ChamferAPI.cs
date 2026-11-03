// Port of: src/ModelingAlgorithms/TKFillet/ChFi2d/ChFi2d_ChamferAPI.hxx
// Port of: src/ModelingAlgorithms/TKFillet/ChFi2d/ChFi2d_ChamferAPI.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2003-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Curves;

using Kerf.Engine.Topology;

using Kerf.Engine.Topology.Brep;

using Kerf.Geometry;

using Kerf.Topology;

namespace Occt;

/// <summary>Makes a chamfer between two linear edges.</summary>
/// <remarks>A working object; its results are new, frozen edges. The edges it is given are only read.</remarks>
public sealed class ChFi2d_ChamferAPI
{
    private Edge myEdge1 = new(), myEdge2 = new();
    private Curve3D? myCurve1, myCurve2;
    private double myStart1, myEnd1, myStart2, myEnd2;
    private bool myCommonStart1, myCommonStart2;

    // An empty constructor.
    public ChFi2d_ChamferAPI()
    {
        myStart1 = 0.0;
        myEnd1 = 0.0;
        myStart2 = 0.0;
        myEnd2 = 0.0;
        myCommonStart1 = false;
        myCommonStart2 = false;
    }

    /// <summary>A chamfer of the first two edges of a wire.</summary>
    public ChFi2d_ChamferAPI(in Wire theWire)
        : this()
    {
        Init(theWire);
    }

    /// <summary>A chamfer of two linear edges.</summary>
    public ChFi2d_ChamferAPI(in Edge theEdge1, in Edge theEdge2)
        : this()
    {
        myEdge1 = theEdge1;
        myEdge2 = theEdge2;
    }

    /// <summary>Initializes the class by a wire consisting of two linear edges.</summary>
    public void Init(in Wire theWire)
    {
        Edge E1 = new(), E2 = new();
        var itr = new Iterator(theWire);
        for (; itr.More(); itr.Next())
        {
            if (E1.IsNull)
            {
                E1 = ShapeCasts.Edge(itr.Value());
            }
            else if (E2.IsNull)
            {
                E2 = ShapeCasts.Edge(itr.Value());
            }
            else
            {
                break;
            }
        }
        Init(E1, E2);
    }

    /// <summary>Initializes the class by two linear edges.</summary>
    public void Init(in Edge theEdge1, in Edge theEdge2)
    {
        myEdge1 = theEdge1;
        myEdge2 = theEdge2;
    }

    /// <summary>Prepares the chamfer: finds the common ends of the edges. Returns true.</summary>
    public bool Perform()
    {
        myCurve1 = Tool.Curve(myEdge1, out myStart1, out myEnd1);
        myCurve2 = Tool.Curve(myEdge2, out myStart2, out myEnd2);
        // searching for common points
        if (myCurve1!.Value(myStart1).IsEqual(myCurve2!.Value(myEnd2), global::Kerf.Engine.Foundation.Precision.Confusion()))
        {
            myCommonStart1 = true;
            myCommonStart2 = false;
        }
        else
        {
            if (myCurve1.Value(myEnd1).IsEqual(myCurve2.Value(myStart2), global::Kerf.Engine.Foundation.Precision.Confusion()))
            {
                myCommonStart1 = false;
                myCommonStart2 = true;
            }
            else
            {
                if (myCurve1.Value(myEnd1).IsEqual(myCurve2.Value(myEnd2), global::Kerf.Engine.Foundation.Precision.Confusion()))
                {
                    myCommonStart1 = false;
                    myCommonStart2 = false;
                }
                else
                {
                    myCommonStart1 = true;
                    myCommonStart2 = true;
                }
            }
        }
        return true;
    }

    /// <summary>
    /// The chamfer edge (frozen), at the parametric lengths <paramref name="theLength1"/> and
    /// <paramref name="theLength2"/> from the common ends, with the two edges trimmed by it; a null edge if an edge is
    /// shorter than its length.
    /// </summary>
    public Edge Result(ref Edge theEdge1, ref Edge theEdge2, double theLength1, double theLength2)
    {
        Edge aResult = new();
        if (Math.Abs(myEnd1 - myStart1) < theLength1)
        {
            return aResult;
        }
        if (Math.Abs(myEnd2 - myStart2) < theLength2)
        {
            return aResult;
        }

        double aCommon1 = (myCommonStart1 ? myStart1 : myEnd1)
                          + (((myStart1 > myEnd1) ^ myCommonStart1) ? theLength1 : -theLength1);
        double aCommon2 = (myCommonStart2 ? myStart2 : myEnd2)
                          + (((myStart2 > myEnd2) ^ myCommonStart2) ? theLength2 : -theLength2);

        // make chamfer edge
        var aML = new MakeLine(myCurve1!.Value(aCommon1), myCurve2!.Value(aCommon2));
        var aBuilder = new BRepBuilderAPI_MakeEdge(aML.Value(),
                                                   myCurve1.Value(aCommon1),
                                                   myCurve2.Value(aCommon2));
        aResult = aBuilder.Edge();
        // divide first edge
        var aDivider1 = new BRepBuilderAPI_MakeEdge(myCurve1, aCommon1, (myCommonStart1 ? myEnd1 : myStart1));
        theEdge1 = aDivider1.Edge();
        // divide second edge
        var aDivider2 = new BRepBuilderAPI_MakeEdge(myCurve2, aCommon2, (myCommonStart2 ? myEnd2 : myStart2));
        theEdge2 = aDivider2.Edge();

        return aResult;
    }
}
