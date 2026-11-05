// Port of: src/ModelingData/TKGeomBase/GeomLib/GeomLib_MakeCurvefromApprox.hxx
// Port of: src/ModelingData/TKGeomBase/GeomLib/GeomLib_MakeCurvefromApprox.lxx
// Port of: src/ModelingData/TKGeomBase/GeomLib/GeomLib_MakeCurvefromApprox.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Approximation.Advanced;
using Kerf.Geometry;
using Kerf.Engine;

namespace Kerf.Engine.Geometry.Library;

/// <summary>
/// Builds 2D and 3D B-spline curves from the results of an AdvApprox approximation: a 3D or 2D subspace, a 3D or 2D
/// subspace divided by a 1D subspace (the weights), or a 2D curve from two 1D subspaces. Each call builds a new curve,
/// which the caller owns.
/// </summary>
internal sealed class MakeCurvefromApprox
{
    // OCCT copies the approximation by value; it is never changed, so the port keeps the caller's object.
    private readonly ApproxAFunction _approx;

    public MakeCurvefromApprox(ApproxAFunction approx)
    {
        _approx = approx;
    }

    public bool IsDone => _approx.IsDone;

    /// <summary>The number of 1D subspaces of the approximation.</summary>
    public int Nb1DSpaces() => _approx.NumSubSpaces(1);

    /// <summary>The number of 2D subspaces of the approximation.</summary>
    public int Nb2DSpaces() => _approx.NumSubSpaces(2);

    /// <summary>The number of 3D subspaces of the approximation.</summary>
    public int Nb3DSpaces() => _approx.NumSubSpaces(3);

    /// <summary>The 2D curve of the subspace Index2d.</summary>
    public BSplineCurve2D Curve2d(int index2d)
    {
        Standard_OutOfRange_Raise_if(index2d < 0 || index2d > Nb2DSpaces(),
                                     " GeomLib_MakeCurvefromApprox : Curve2d");
        StdFail_NotDone_Raise_if(!IsDone, " GeomLib_MakeCurvefromApprox : Curve2d");

        NCollection_Array1<Point2D> poles = new(1, _approx.NbPoles());
        NCollection_Array1<double> knots = new(1, _approx.KnotCount);
        NCollection_Array1<int> mults = new(1, _approx.KnotCount);

        _approx.Poles2d(index2d, poles);
        knots.Assign(_approx.Knots()!.Array1());
        mults.Assign(_approx.Multiplicities()!.Array1());

        BSplineCurve2D c = new(poles, knots, mults, _approx.Degree());

        return c;
    }

    /// <summary>The 2D curve of the subspace Index2d divided by the 1D subspace Index1d (the weights).</summary>
    public BSplineCurve2D Curve2d(int index1d, int index2d)
    {
        Standard_OutOfRange_Raise_if(index1d < 0 || index1d > Nb1DSpaces() || index2d < 0
                                       || index2d > Nb2DSpaces(),
                                     " GeomLib_MakeCurvefromApprox : Curve2d");
        StdFail_NotDone_Raise_if(!IsDone && !_approx.HasResult,
                                 " GeomLib_MakeCurvefromApprox : Curve2d");

        NCollection_Array1<Point2D> poles = new(1, _approx.NbPoles());
        NCollection_Array1<double> weigths = new(1, _approx.NbPoles());
        NCollection_Array1<double> knots = new(1, _approx.KnotCount);
        NCollection_Array1<int> mults = new(1, _approx.KnotCount);

        _approx.Poles2d(index2d, poles);
        _approx.Poles1d(index1d, weigths);
        knots.Assign(_approx.Knots()!.Array1());
        mults.Assign(_approx.Multiplicities()!.Array1());

        double x, y, w;
        for (int i = 1; i <= _approx.NbPoles(); i++)
        {
            w = weigths[i];
            poles[i].Coord(out x, out y);
            poles[i] = poles[i].WithCoord(x / w, y / w);
        }

        BSplineCurve2D c = new(poles, knots, mults, _approx.Degree());

        return c;
    }

    /// <summary>The 2D curve whose X and Y are the 1D subspaces Index1d and Index2d.</summary>
    public BSplineCurve2D Curve2dFromTwo1d(int index1d, int index2d)
    {
        Standard_OutOfRange_Raise_if(index1d < 0 || index1d > Nb1DSpaces() || index2d < 0
                                       || index2d > Nb1DSpaces(),
                                     " GeomLib_MakeCurvefromApprox : Curve2d");
        StdFail_NotDone_Raise_if(!IsDone && !_approx.HasResult,
                                 " GeomLib_MakeCurvefromApprox : Curve2d");

        NCollection_Array1<Point2D> poles = new(1, _approx.NbPoles());
        NCollection_Array1<double> poles1d1 = new(1, _approx.NbPoles());
        NCollection_Array1<double> poles1d2 = new(1, _approx.NbPoles());
        NCollection_Array1<double> knots = new(1, _approx.KnotCount);
        NCollection_Array1<int> mults = new(1, _approx.KnotCount);

        _approx.Poles1d(index2d, poles1d2);
        _approx.Poles1d(index1d, poles1d1);

        knots.Assign(_approx.Knots()!.Array1());
        mults.Assign(_approx.Multiplicities()!.Array1());

        for (int i = 1; i <= _approx.NbPoles(); i++)
        {
            poles[i] = poles[i].WithCoord(poles1d1.Value(i), poles1d2.Value(i));
        }

        BSplineCurve2D c = new(poles, knots, mults, _approx.Degree());

        return c;
    }

    /// <summary>The 3D curve of the subspace Index3d.</summary>
    public BSplineCurve3D Curve(int index3d)
    {
        Standard_OutOfRange_Raise_if(index3d < 0 || index3d > Nb3DSpaces(),
                                     " GeomLib_MakeCurvefromApprox : Curve");
        StdFail_NotDone_Raise_if(!IsDone && !_approx.HasResult,
                                 " GeomLib_MakeCurvefromApprox : Curve");

        NCollection_Array1<Point3D> poles = new(1, _approx.NbPoles());
        NCollection_Array1<double> knots = new(1, _approx.KnotCount);
        NCollection_Array1<int> mults = new(1, _approx.KnotCount);

        _approx.Poles(index3d, poles);
        knots.Assign(_approx.Knots()!.Array1());
        mults.Assign(_approx.Multiplicities()!.Array1());

        BSplineCurve3D c = new(poles, knots, mults, _approx.Degree());

        return c;
    }

    /// <summary>The 3D curve of the subspace Index3d divided by the 1D subspace Index1d (the weights).</summary>
    public BSplineCurve3D Curve(int index1d, int index3d)
    {
        Standard_OutOfRange_Raise_if(index1d < 0 || index1d > Nb1DSpaces() || index3d < 0
                                       || index3d > Nb3DSpaces(),
                                     " GeomLib_MakeCurvefromApprox : Curve3d");
        StdFail_NotDone_Raise_if(!IsDone, " GeomLib_MakeCurvefromApprox : Curve3d");

        NCollection_Array1<Point3D> poles = new(1, _approx.NbPoles());
        NCollection_Array1<double> weigths = new(1, _approx.NbPoles());
        NCollection_Array1<double> knots = new(1, _approx.KnotCount);
        NCollection_Array1<int> mults = new(1, _approx.KnotCount);

        _approx.Poles(index3d, poles);
        _approx.Poles1d(index1d, weigths);
        knots.Assign(_approx.Knots()!.Array1());
        mults.Assign(_approx.Multiplicities()!.Array1());

        double x, y, z, w;
        for (int i = 1; i <= _approx.NbPoles(); i++)
        {
            w = weigths[i];
            poles[i].Coord(out x, out y, out z);
            poles[i] = poles[i].WithCoord(x / w, y / w, z / w);
        }

        BSplineCurve3D c = new(poles, knots, mults, _approx.Degree());

        return c;
    }
}
