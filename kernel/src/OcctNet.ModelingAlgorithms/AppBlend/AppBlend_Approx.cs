// Port of: src/ModelingAlgorithms/TKGeomAlgo/AppBlend/AppBlend_Approx.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/AppBlend/AppBlend_Approx.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

namespace Occt;

/// <summary>The B-spline approximation of a surface, with its curves on the surface (the root of the AppSurf classes).</summary>
/// <remarks>
/// The arrays the accessors return (<see cref="SurfPoles"/>, <see cref="SurfUKnots"/>, ...) are the object's own,
/// as OCCT returns them by const reference (decision 143): shared, don't modify. <see cref="Surface"/> and
/// <see cref="Curve2d"/> copy them into the caller's arrays.
/// </remarks>
public abstract class AppBlend_Approx
{
    public abstract bool IsDone();

    public abstract void SurfShape(out int UDegree, out int VDegree, out int NbUPoles, out int NbVPoles,
                                   out int NbUKnots, out int NbVKnots);

    public abstract void Surface(NCollection_Array2<Point3D> TPoles, NCollection_Array2<double> TWeights,
                                 NCollection_Array1<double> TUKnots, NCollection_Array1<double> TVKnots,
                                 NCollection_Array1<int> TUMults, NCollection_Array1<int> TVMults);

    public abstract int UDegree();

    public abstract int VDegree();

    public abstract NCollection_Array2<Point3D> SurfPoles();

    public abstract NCollection_Array2<double> SurfWeights();

    public abstract NCollection_Array1<double> SurfUKnots();

    public abstract NCollection_Array1<double> SurfVKnots();

    public abstract NCollection_Array1<int> SurfUMults();

    public abstract NCollection_Array1<int> SurfVMults();

    public abstract int NbCurves2d();

    public abstract void Curves2dShape(out int Degree, out int NbPoles, out int NbKnots);

    public abstract void Curve2d(int Index, NCollection_Array1<Point2D> TPoles, NCollection_Array1<double> TKnots,
                                 NCollection_Array1<int> TMults);

    public abstract int Curves2dDegree();

    public abstract NCollection_Array1<Point2D> Curve2dPoles(int Index);

    public abstract NCollection_Array1<double> Curves2dKnots();

    public abstract NCollection_Array1<int> Curves2dMults();

    public abstract void TolReached(out double Tol3d, out double Tol2d);

    public abstract double TolCurveOnSurf(int Index);
}
