// Port of: src/ModelingAlgorithms/TKGeomAlgo/AppBlend/AppBlend_Approx.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/AppBlend/AppBlend_Approx.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using Kerf.Engine;

namespace Kerf.Engine.Fillets.Blend;

/// <summary>The B-spline approximation of a surface, with its curves on the surface (the root of the AppSurf classes).</summary>
/// <remarks>
/// The arrays the accessors return (<see cref="SurfPoles"/>, <see cref="SurfUKnots"/>, ...) are the object's own,
/// as OCCT returns them by const reference (decision 143): shared, don't modify. <see cref="Surface"/> and
/// <see cref="Curve2d"/> copy them into the caller's arrays.
/// </remarks>
internal abstract class Approx
{
    public abstract bool IsDone { get; }

    public abstract void SurfShape(out int uDegree, out int vDegree, out int nbUPoles, out int nbVPoles,
                                   out int nbUKnots, out int nbVKnots);

    public abstract void Surface(NCollection_Array2<Point3D> tPoles, NCollection_Array2<double> tWeights,
                                 NCollection_Array1<double> tuKnots, NCollection_Array1<double> tvKnots,
                                 NCollection_Array1<int> tuMults, NCollection_Array1<int> tvMults);

    public abstract int UDegree();

    public abstract int VDegree();

    public abstract NCollection_Array2<Point3D> SurfPoles();

    public abstract NCollection_Array2<double> SurfWeights();

    public abstract NCollection_Array1<double> SurfUKnots();

    public abstract NCollection_Array1<double> SurfVKnots();

    public abstract NCollection_Array1<int> SurfUMults();

    public abstract NCollection_Array1<int> SurfVMults();

    public abstract int Curves2dCount { get; }

    public abstract void Curves2dShape(out int degree, out int nbPoles, out int nbKnots);

    public abstract void Curve2d(int index, NCollection_Array1<Point2D> tPoles, NCollection_Array1<double> tKnots,
                                 NCollection_Array1<int> tMults);

    public abstract int Curves2dDegree();

    public abstract NCollection_Array1<Point2D> Curve2dPoles(int index);

    public abstract NCollection_Array1<double> Curves2dKnots();

    public abstract NCollection_Array1<int> Curves2dMults();

    public abstract void TolReached(out double tol3d, out double tol2d);

    public abstract double TolCurveOnSurf(int index);
}
