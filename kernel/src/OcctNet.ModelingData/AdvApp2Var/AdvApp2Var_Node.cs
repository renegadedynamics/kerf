// Port of: src/ModelingData/TKGeomBase/AdvApp2Var/AdvApp2Var_Node.hxx
// Port of: src/ModelingData/TKGeomBase/AdvApp2Var/AdvApp2Var_Node.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry;

using Kerf.Geometry;

namespace Occt;

/// <summary>
/// The constraints of <see cref="AdvApp2Var_ApproxAFunc2Var"/> at a node (Ui, Vj): the values of F and of its
/// derivatives up to the continuity orders, and the errors of their approximation.
/// </summary>
/// <remarks>A working object of the approximation, with OCCT's public mutators (decision 194).</remarks>
public class AdvApp2Var_Node : Transient
{
    private readonly NCollection_Array2<Point3D> myTruePoints;
    private readonly NCollection_Array2<double> myErrors;
    private Xy myCoord;
    private int myOrdInU;
    private int myOrdInV;

    private static void initNodeFields(NCollection_Array2<Point3D> theTruePoints, NCollection_Array2<double> theErrors)
    {
        Point3D aZeroPoint = new(0.0, 0.0, 0.0);
        theTruePoints.Init(aZeroPoint);
        theErrors.Init(0.0);
    }

    public AdvApp2Var_Node()
    {
        myTruePoints = new NCollection_Array2<Point3D>(0, 2, 0, 2);
        myErrors = new NCollection_Array2<double>(0, 2, 0, 2);
        myOrdInU = 2;
        myOrdInV = 2;
        initNodeFields(myTruePoints, myErrors);
    }

    public AdvApp2Var_Node(int iu, int iv)
    {
        myTruePoints = new NCollection_Array2<Point3D>(0, Max(0, iu), 0, Max(0, iv));
        myErrors = new NCollection_Array2<double>(0, Max(0, iu), 0, Max(0, iv));
        myOrdInU = iu;
        myOrdInV = iv;
        initNodeFields(myTruePoints, myErrors);
    }

    public AdvApp2Var_Node(in Xy UV, int iu, int iv)
    {
        myTruePoints = new NCollection_Array2<Point3D>(0, iu, 0, iv);
        myErrors = new NCollection_Array2<double>(0, iu, 0, iv);
        myCoord = UV;
        myOrdInU = iu;
        myOrdInV = iv;
        initNodeFields(myTruePoints, myErrors);
    }

    /// <summary>The coordinates (U, V) of the node.</summary>
    public Xy Coord() => myCoord;

    /// <summary>Changes the coordinates (U, V) to (<paramref name="x1"/>, <paramref name="x2"/>).</summary>
    public void SetCoord(double x1, double x2)
    {
        myCoord.SetX(x1);
        myCoord.SetY(x2);
    }

    /// <summary>The continuity order in U of the node.</summary>
    public int UOrder() => myOrdInU;

    /// <summary>The continuity order in V of the node.</summary>
    public int VOrder() => myOrdInV;

    /// <summary>Sets F(U, V), or its derivative of order (<paramref name="iu"/>, <paramref name="iv"/>), at the node.</summary>
    public void SetPoint(int iu, int iv, in Point3D Pt) => myTruePoints.SetValue(iu, iv, Pt);

    /// <summary>F(U, V), or its derivative of order (<paramref name="iu"/>, <paramref name="iv"/>), at the node.</summary>
    public Point3D Point(int iu, int iv) => myTruePoints.Value(iu, iv);

    /// <summary>Sets the error between F(U, V) and its approximation.</summary>
    public void SetError(int iu, int iv, double error) => myErrors.SetValue(iu, iv, error);

    /// <summary>The error between F(U, V) and its approximation.</summary>
    public double Error(int iu, int iv) => myErrors.Value(iu, iv);

    /// <summary>Copies another node's values into this one (C++: <c>operator=</c>).</summary>
    public AdvApp2Var_Node Assign(AdvApp2Var_Node theOther)
    {
        myTruePoints.Assign(theOther.myTruePoints);
        myErrors.Assign(theOther.myErrors);
        myCoord = theOther.myCoord;
        myOrdInU = theOther.myOrdInU;
        myOrdInV = theOther.myOrdInV;
        return this;
    }
}
