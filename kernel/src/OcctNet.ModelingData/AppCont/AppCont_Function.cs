// Port of: src/ModelingData/TKGeomBase/AppCont/AppCont_Function.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// A continuous function f(u) whose values are a set of 3d and/or 2d points: what the continuous approximations
/// (AppCont_LeastSquare, Approx_FitAndDivide) approximate.
/// </summary>
/// <remarks>
/// An implementation sets <see cref="myNbPnt"/> and <see cref="myNbPnt2d"/> in its constructor (both are -1
/// until then, as in OCCT), and fills the arrays passed to <see cref="Value"/> and <see cref="D1"/> from index 1:
/// the 3d points (or vectors) in one array, the 2d ones in the other. The arrays are allocated by the caller with at
/// least one element each, even when there are no points of that dimension.
/// </remarks>
public abstract class AppCont_Function
{
    /// <summary>The number of 3d points returned by Value and D1.</summary>
    protected int myNbPnt;

    /// <summary>The number of 2d points returned by Value and D1.</summary>
    protected int myNbPnt2d;

    protected AppCont_Function()
    {
        myNbPnt = -1;
        myNbPnt2d = -1;
    }

    /// <summary>The number of 3d and 2d points returned by <see cref="Value"/> and <see cref="D1"/>.</summary>
    public void GetNumberOfPoints(out int theNbPnt, out int theNbPnt2d)
    {
        theNbPnt = myNbPnt;
        theNbPnt2d = myNbPnt2d;
    }

    /// <summary>The number of 3d points returned by <see cref="Value"/> and <see cref="D1"/>.</summary>
    public int GetNbOf3dPoints() => myNbPnt;

    /// <summary>The number of 2d points returned by <see cref="Value"/> and <see cref="D1"/>.</summary>
    public int GetNbOf2dPoints() => myNbPnt2d;

    /// <summary>The first parameter of the function.</summary>
    public abstract double FirstParameter();

    /// <summary>The last parameter of the function.</summary>
    public abstract double LastParameter();

    /// <summary>Fills the arrays with the points at <paramref name="theU"/>; false if they can't be computed.</summary>
    public abstract bool Value(double theU, NCollection_Array1<gp_Pnt2d> thePnt2d, NCollection_Array1<gp_Pnt> thePnt);

    /// <summary>Fills the arrays with the derivatives at <paramref name="theU"/>; false if they can't be computed.</summary>
    public abstract bool D1(double theU, NCollection_Array1<gp_Vec2d> theVec2d, NCollection_Array1<gp_Vec> theVec);

    /// <summary>
    /// The periodicity of the output coordinate <paramref name="theDimIdx"/> (1 to 3 * NbPnt + 2 * NbPnt2d: the
    /// coordinates of the 3d points, then of the 2d points). Not periodic by default.
    /// </summary>
    public virtual void PeriodInformation(int theDimIdx, out bool IsPeriodic, out double thePeriod)
    {
        IsPeriodic = false;
        thePeriod = 0.0;
    }
}
