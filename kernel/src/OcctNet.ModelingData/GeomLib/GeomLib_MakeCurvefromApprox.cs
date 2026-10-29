// Port of: src/ModelingData/TKGeomBase/GeomLib/GeomLib_MakeCurvefromApprox.hxx
// Port of: src/ModelingData/TKGeomBase/GeomLib/GeomLib_MakeCurvefromApprox.lxx
// Port of: src/ModelingData/TKGeomBase/GeomLib/GeomLib_MakeCurvefromApprox.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// Builds 2D and 3D B-spline curves from the results of an AdvApprox approximation: a 3D or 2D subspace, a 3D or 2D
/// subspace divided by a 1D subspace (the weights), or a 2D curve from two 1D subspaces. Each call builds a new curve,
/// which the caller owns.
/// </summary>
public sealed class GeomLib_MakeCurvefromApprox
{
    // OCCT copies the approximation by value; it is never changed, so the port keeps the caller's object.
    private readonly AdvApprox_ApproxAFunction myApprox;

    public GeomLib_MakeCurvefromApprox(AdvApprox_ApproxAFunction Approx)
    {
        myApprox = Approx;
    }

    public bool IsDone() => myApprox.IsDone();

    /// <summary>The number of 1D subspaces of the approximation.</summary>
    public int Nb1DSpaces() => myApprox.NumSubSpaces(1);

    /// <summary>The number of 2D subspaces of the approximation.</summary>
    public int Nb2DSpaces() => myApprox.NumSubSpaces(2);

    /// <summary>The number of 3D subspaces of the approximation.</summary>
    public int Nb3DSpaces() => myApprox.NumSubSpaces(3);

    /// <summary>The 2D curve of the subspace Index2d.</summary>
    public Geom2d_BSplineCurve Curve2d(int Index2d)
    {
        Standard_OutOfRange_Raise_if(Index2d < 0 || Index2d > Nb2DSpaces(),
                                     " GeomLib_MakeCurvefromApprox : Curve2d");
        StdFail_NotDone_Raise_if(!IsDone(), " GeomLib_MakeCurvefromApprox : Curve2d");

        NCollection_Array1<gp_Pnt2d> Poles = new(1, myApprox.NbPoles());
        NCollection_Array1<double> Knots = new(1, myApprox.NbKnots());
        NCollection_Array1<int> Mults = new(1, myApprox.NbKnots());

        myApprox.Poles2d(Index2d, Poles);
        Knots.Assign(myApprox.Knots()!.Array1());
        Mults.Assign(myApprox.Multiplicities()!.Array1());

        Geom2d_BSplineCurve C = new(Poles, Knots, Mults, myApprox.Degree());

        return C;
    }

    /// <summary>The 2D curve of the subspace Index2d divided by the 1D subspace Index1d (the weights).</summary>
    public Geom2d_BSplineCurve Curve2d(int Index1d, int Index2d)
    {
        Standard_OutOfRange_Raise_if(Index1d < 0 || Index1d > Nb1DSpaces() || Index2d < 0
                                       || Index2d > Nb2DSpaces(),
                                     " GeomLib_MakeCurvefromApprox : Curve2d");
        StdFail_NotDone_Raise_if(!IsDone() && !myApprox.HasResult(),
                                 " GeomLib_MakeCurvefromApprox : Curve2d");

        NCollection_Array1<gp_Pnt2d> Poles = new(1, myApprox.NbPoles());
        NCollection_Array1<double> Weigths = new(1, myApprox.NbPoles());
        NCollection_Array1<double> Knots = new(1, myApprox.NbKnots());
        NCollection_Array1<int> Mults = new(1, myApprox.NbKnots());

        myApprox.Poles2d(Index2d, Poles);
        myApprox.Poles1d(Index1d, Weigths);
        Knots.Assign(myApprox.Knots()!.Array1());
        Mults.Assign(myApprox.Multiplicities()!.Array1());

        double X, Y, W;
        for (int i = 1; i <= myApprox.NbPoles(); i++)
        {
            W = Weigths[i];
            Poles[i].Coord(out X, out Y);
            Poles[i].SetCoord(X / W, Y / W);
        }

        Geom2d_BSplineCurve C = new(Poles, Knots, Mults, myApprox.Degree());

        return C;
    }

    /// <summary>The 2D curve whose X and Y are the 1D subspaces Index1d and Index2d.</summary>
    public Geom2d_BSplineCurve Curve2dFromTwo1d(int Index1d, int Index2d)
    {
        Standard_OutOfRange_Raise_if(Index1d < 0 || Index1d > Nb1DSpaces() || Index2d < 0
                                       || Index2d > Nb1DSpaces(),
                                     " GeomLib_MakeCurvefromApprox : Curve2d");
        StdFail_NotDone_Raise_if(!IsDone() && !myApprox.HasResult(),
                                 " GeomLib_MakeCurvefromApprox : Curve2d");

        NCollection_Array1<gp_Pnt2d> Poles = new(1, myApprox.NbPoles());
        NCollection_Array1<double> Poles1d1 = new(1, myApprox.NbPoles());
        NCollection_Array1<double> Poles1d2 = new(1, myApprox.NbPoles());
        NCollection_Array1<double> Knots = new(1, myApprox.NbKnots());
        NCollection_Array1<int> Mults = new(1, myApprox.NbKnots());

        myApprox.Poles1d(Index2d, Poles1d2);
        myApprox.Poles1d(Index1d, Poles1d1);

        Knots.Assign(myApprox.Knots()!.Array1());
        Mults.Assign(myApprox.Multiplicities()!.Array1());

        for (int i = 1; i <= myApprox.NbPoles(); i++)
        {
            Poles[i].SetCoord(Poles1d1.Value(i), Poles1d2.Value(i));
        }

        Geom2d_BSplineCurve C = new(Poles, Knots, Mults, myApprox.Degree());

        return C;
    }

    /// <summary>The 3D curve of the subspace Index3d.</summary>
    public Geom_BSplineCurve Curve(int Index3d)
    {
        Standard_OutOfRange_Raise_if(Index3d < 0 || Index3d > Nb3DSpaces(),
                                     " GeomLib_MakeCurvefromApprox : Curve");
        StdFail_NotDone_Raise_if(!IsDone() && !myApprox.HasResult(),
                                 " GeomLib_MakeCurvefromApprox : Curve");

        NCollection_Array1<gp_Pnt> Poles = new(1, myApprox.NbPoles());
        NCollection_Array1<double> Knots = new(1, myApprox.NbKnots());
        NCollection_Array1<int> Mults = new(1, myApprox.NbKnots());

        myApprox.Poles(Index3d, Poles);
        Knots.Assign(myApprox.Knots()!.Array1());
        Mults.Assign(myApprox.Multiplicities()!.Array1());

        Geom_BSplineCurve C = new(Poles, Knots, Mults, myApprox.Degree());

        return C;
    }

    /// <summary>The 3D curve of the subspace Index3d divided by the 1D subspace Index1d (the weights).</summary>
    public Geom_BSplineCurve Curve(int Index1d, int Index3d)
    {
        Standard_OutOfRange_Raise_if(Index1d < 0 || Index1d > Nb1DSpaces() || Index3d < 0
                                       || Index3d > Nb3DSpaces(),
                                     " GeomLib_MakeCurvefromApprox : Curve3d");
        StdFail_NotDone_Raise_if(!IsDone(), " GeomLib_MakeCurvefromApprox : Curve3d");

        NCollection_Array1<gp_Pnt> Poles = new(1, myApprox.NbPoles());
        NCollection_Array1<double> Weigths = new(1, myApprox.NbPoles());
        NCollection_Array1<double> Knots = new(1, myApprox.NbKnots());
        NCollection_Array1<int> Mults = new(1, myApprox.NbKnots());

        myApprox.Poles(Index3d, Poles);
        myApprox.Poles1d(Index1d, Weigths);
        Knots.Assign(myApprox.Knots()!.Array1());
        Mults.Assign(myApprox.Multiplicities()!.Array1());

        double X, Y, Z, W;
        for (int i = 1; i <= myApprox.NbPoles(); i++)
        {
            W = Weigths[i];
            Poles[i].Coord(out X, out Y, out Z);
            Poles[i].SetCoord(X / W, Y / W, Z / W);
        }

        Geom_BSplineCurve C = new(Poles, Knots, Mults, myApprox.Degree());

        return C;
    }
}
