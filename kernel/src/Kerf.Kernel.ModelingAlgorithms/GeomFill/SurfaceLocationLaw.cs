// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_LocationLaw.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_LocationLaw.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Geometry;
using Kerf.Engine;

namespace Kerf.Engine.Construction.Surfaces;

/// <summary>
/// A location law for sweeping: a matrix M(t) and a vector V(t), which move a point P to M P + V.
/// </summary>
/// <remarks>
/// A working object (decision 21), as <see cref="TrihedronLaw"/>: <see cref="SetCurve"/>,
/// <see cref="SetTrsf"/>, <see cref="SetInterval"/> and <see cref="SetTolerance"/> are public and change the law.
/// The evaluators' outputs are <c>ref</c>: some laws leave them unwritten on some paths, as in OCCT.
/// </remarks>
internal abstract class SurfaceLocationLaw : Transient
{
    /// <summary>Initializes the curve of the location law.</summary>
    public abstract bool SetCurve(CurveAdaptorBase c);

    public abstract CurveAdaptorBase? GetCurve();

    /// <summary>Sets a transformation matrix, so that the law M(t) becomes Mat * M(t).</summary>
    public abstract void SetTrsf(in Matrix3x3 transfo);

    public abstract SurfaceLocationLaw Copy();

    /// <summary>Computes the location.</summary>
    public abstract bool D0(double param, ref Matrix3x3 m, ref Vector3D v);

    /// <summary>Computes the location and the 2D points.</summary>
    public abstract bool D0(double param, ref Matrix3x3 m, ref Vector3D v, NCollection_Array1<Point2D> poles2d);

    /// <summary>Computes the location, the 2D points and their first derivatives (used only for C1 or C2 approximation).</summary>
    public virtual bool D1(double param, ref Matrix3x3 m, ref Vector3D v, ref Matrix3x3 dm, ref Vector3D dv,
                           NCollection_Array1<Point2D> poles2d, NCollection_Array1<Vector2D> dPoles2d)
    {
        throw new MethodNotImplementedException("GeomFill_LocationLaw::D1");
    }

    /// <summary>Computes the location, the 2D points and their first and second derivatives (used only for C2 approximation).</summary>
    public virtual bool D2(double param, ref Matrix3x3 m, ref Vector3D v, ref Matrix3x3 dm, ref Vector3D dv, ref Matrix3x3 d2M,
                           ref Vector3D d2V, NCollection_Array1<Point2D> poles2d, NCollection_Array1<Vector2D> dPoles2d,
                           NCollection_Array1<Vector2D> d2Poles2d)
    {
        throw new MethodNotImplementedException("GeomFill_LocationLaw::D2");
    }

    /// <summary>The number of 2D curves (restrictions and traces) to approximate.</summary>
    public int Nb2dCurves()
    {
        int n = TraceNumber();
        if (HasFirstRestriction)
        {
            n++;
        }
        if (HasLastRestriction)
        {
            n++;
        }
        return n;
    }

    /// <summary>Whether the first restriction is defined in this law (the first 2D pole is then its); false by default.</summary>
    public virtual bool HasFirstRestriction => false;

    /// <summary>Whether the last restriction is defined in this law (the last 2D pole is then its); false by default.</summary>
    public virtual bool HasLastRestriction => false;

    /// <summary>The number of traces (2D curves that are not restrictions); 0 by default.</summary>
    public virtual int TraceNumber() => 0;

    /// <summary>The status of the law; PipeOk by default.</summary>
    public virtual SurfacePipeError ErrorStatus() => SurfacePipeError.PipeOk;

    /// <summary>The number of intervals for continuity S; may be one if the law is at least S.</summary>
    public abstract int NbIntervals(Continuity s);

    /// <summary>Stores in T the parameters bounding the intervals of continuity S; T.Length() must exceed NbIntervals(S).</summary>
    public abstract void Intervals(NCollection_Array1<double> t, Continuity s);

    /// <summary>Sets the bounds of the parametric interval, which determines the derivatives there if the law is not Cn.</summary>
    public abstract void SetInterval(double first, double last);

    /// <summary>The bounds of the parametric interval.</summary>
    public abstract void GetInterval(out double first, out double last);

    /// <summary>The bounds of the parametric domain of the law (not changed by SetInterval).</summary>
    public abstract void GetDomain(out double first, out double last);

    /// <summary>The resolutions in the 2D sub-space Index, to choose a 2D approximation tolerance.</summary>
    public virtual void Resolution(int index, double tol, out double tolU, out double tolV)
    {
        throw new MethodNotImplementedException("GeomFill_LocationLaw::Resolution");
    }

    /// <summary>Sets the tolerances of a law that runs numerical algorithms; does nothing by default.</summary>
    public virtual void SetTolerance(double tol3d, double tol2d)
    {
        // Ne fait rien !!
    }

    /// <summary>The maximum norm of the matrix part, to choose a tolerance to approximate M(t).</summary>
    public abstract double GetMaximalNorm();

    /// <summary>The average of M(t) and V(t), useful to make fast approximations of rational surfaces.</summary>
    public abstract void GetAverageLaw(ref Matrix3x3 am, ref Vector3D av);

    /// <summary>Whether the law is a translation; false by default.</summary>
    public virtual bool IsTranslation(ref double error) => false;

    /// <summary>Whether the law is a rotation; false by default.</summary>
    public virtual bool IsRotation(ref double error) => false;

    public virtual void Rotation(ref Point3D center)
    {
        throw new MethodNotImplementedException("GeomFill_SectionLaw::Rotation");
    }
}
