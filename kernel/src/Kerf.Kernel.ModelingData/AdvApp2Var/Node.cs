// Port of: src/ModelingData/TKGeomBase/AdvApp2Var/AdvApp2Var_Node.hxx
// Port of: src/ModelingData/TKGeomBase/AdvApp2Var/AdvApp2Var_Node.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry;

using Kerf.Geometry;
using Kerf.Engine;

namespace Kerf.Engine.Approximation.Advanced;

/// <summary>
/// The constraints of <see cref="ApproxAFunc2Var"/> at a node (Ui, Vj): the values of F and of its
/// derivatives up to the continuity orders, and the errors of their approximation.
/// </summary>
/// <remarks>A working object of the approximation, with OCCT's public mutators (decision 194).</remarks>
internal class Node : Transient
{
    private readonly NCollection_Array2<Point3D> _truePoints;
    private readonly NCollection_Array2<double> _errors;
    private Xy _coord;
    private int _ordInU;
    private int _ordInV;

    private static void InitNodeFields(NCollection_Array2<Point3D> truePoints, NCollection_Array2<double> errors)
    {
        Point3D zeroPoint = new(0.0, 0.0, 0.0);
        truePoints.Init(zeroPoint);
        errors.Init(0.0);
    }

    public Node()
    {
        _truePoints = new NCollection_Array2<Point3D>(0, 2, 0, 2);
        _errors = new NCollection_Array2<double>(0, 2, 0, 2);
        _ordInU = 2;
        _ordInV = 2;
        InitNodeFields(_truePoints, _errors);
    }

    public Node(int iu, int iv)
    {
        _truePoints = new NCollection_Array2<Point3D>(0, Max(0, iu), 0, Max(0, iv));
        _errors = new NCollection_Array2<double>(0, Max(0, iu), 0, Max(0, iv));
        _ordInU = iu;
        _ordInV = iv;
        InitNodeFields(_truePoints, _errors);
    }

    public Node(in Xy uv, int iu, int iv)
    {
        _truePoints = new NCollection_Array2<Point3D>(0, iu, 0, iv);
        _errors = new NCollection_Array2<double>(0, iu, 0, iv);
        _coord = uv;
        _ordInU = iu;
        _ordInV = iv;
        InitNodeFields(_truePoints, _errors);
    }

    /// <summary>The coordinates (U, V) of the node.</summary>
    public Xy Coord() => _coord;

    /// <summary>Changes the coordinates (U, V) to (<paramref name="x1"/>, <paramref name="x2"/>).</summary>
    public void SetCoord(double x1, double x2)
    {
        _coord.SetX(x1);
        _coord.SetY(x2);
    }

    /// <summary>The continuity order in U of the node.</summary>
    public int UOrder() => _ordInU;

    /// <summary>The continuity order in V of the node.</summary>
    public int VOrder() => _ordInV;

    /// <summary>Sets F(U, V), or its derivative of order (<paramref name="iu"/>, <paramref name="iv"/>), at the node.</summary>
    public void SetPoint(int iu, int iv, in Point3D pt) => _truePoints.SetValue(iu, iv, pt);

    /// <summary>F(U, V), or its derivative of order (<paramref name="iu"/>, <paramref name="iv"/>), at the node.</summary>
    public Point3D Point(int iu, int iv) => _truePoints.Value(iu, iv);

    /// <summary>Sets the error between F(U, V) and its approximation.</summary>
    public void SetError(int iu, int iv, double error) => _errors.SetValue(iu, iv, error);

    /// <summary>The error between F(U, V) and its approximation.</summary>
    public double Error(int iu, int iv) => _errors.Value(iu, iv);

    /// <summary>Copies another node's values into this one (C++: <c>operator=</c>).</summary>
    public Node Assign(Node other)
    {
        _truePoints.Assign(other._truePoints);
        _errors.Assign(other._errors);
        _coord = other._coord;
        _ordInU = other._ordInU;
        _ordInV = other._ordInV;
        return this;
    }
}
