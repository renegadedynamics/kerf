// Port of: src/ModelingData/TKGeomBase/GC/GC_MakeArcOfCircle.hxx
// Port of: src/ModelingData/TKGeomBase/GC/GC_MakeArcOfCircle.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Values;
using Kerf.Engine.Extrema;
using Kerf.Engine.Foundation;
using Kerf.Geometry;
using Occt;
using static Kerf.Engine.Construction.Values.ErrorType;

namespace Kerf.Engine.Construction.Curves;

/// <summary>
/// Constructions of an arc of circle in 3D as a Geom_TrimmedCurve: through three points, from two points and the
/// tangent at the first, or on a circle between two points or parameters.
/// </summary>
internal sealed class MakeArcOfCircle : Root
{
    private TrimmedCurve3D? _theArc;

    /// <summary>The arc through P1, P2 and P3, from P1 to P3; fails with the status of gce_MakeCirc(P1, P2, P3).</summary>
    public MakeArcOfCircle(in Point3D p1, in Point3D p2, in Point3D p3)
    {
        bool sense;
        //
        MakeCirc cir = new MakeCirc(p1, p2, p3);
        _theError = cir.Status();
        if (_theError == Done)
        {
            double alpha1, alpha3; //,Alpha2
            Circle3D c = cir.Value();
            // modified by NIZNHY-PKV Thu Mar  3 10:53:02 2005: Alpha1 is always 0 (OCCT computed the three
            // parameters and the sense before).
            alpha1 = 0.0;
            alpha3 = global::Kerf.Engine.Geometry.ElCLib.Parameter(c, p3);
            sense = true;

            CircleCurve3D circ = new CircleCurve3D(c);
            _theArc = new TrimmedCurve3D(circ, alpha1, alpha3, sense);
        }
    }

    /// <summary>
    /// The arc from P1 to P2 tangent to V at P1. Fails with gce_ConfusedPoints if P1 and P2 coincide, and with
    /// gce_IntersectionError if the lines defining the center don't intersect.
    /// </summary>
    public MakeArcOfCircle(in Point3D p1, in Vector3D v, in Point3D p2)
    {
        Circle3D cir;
        MakeLin Corde = new MakeLin(p1, p2);
        _theError = Corde.Status();
        if (_theError == Done)
        {
            Line3D corde = Corde.Value();
            Direction3D dir = corde.Direction;
            Direction3D dbid = new Direction3D(v);
            Direction3D daxe = dbid ^ dir;
            Direction3D dir1 = daxe ^ dir;
            Line3D bis = new Line3D(
                new Point3D((p1.X + p2.X) / 2.0, (p1.Y + p2.Y) / 2.0, (p1.Z + p2.Z) / 2.0), dir1);
            Direction3D d = dbid ^ daxe;
            Line3D norm = new Line3D(p1, d);
            ExtElC distmin = new ExtElC(bis, norm, global::Kerf.Engine.Foundation.Precision.Confusion());
            if (!distmin.IsDone)
            {
                _theError = IntersectionError;
            }
            else
            {
                int nbext = distmin.ExtCount;
                if (nbext == 0)
                {
                    _theError = IntersectionError;
                }
                else
                {
                    double theDist = RealLast();
                    Point3D pInt = new(), pon1, pon2;
                    int i = 1;
                    while (i <= nbext)
                    {
                        if (distmin.SquareDistance(i) < theDist)
                        {
                            theDist = distmin.SquareDistance(i);
                            distmin.Points(i, out POnCurv Pon1, out POnCurv Pon2);
                            pon1 = Pon1.Value();
                            pon2 = Pon2.Value();
                            pInt = new Point3D((pon1.XYZ() + pon2.XYZ()) / 2.0);
                        }
                        i++;
                    }
                    double rad = (pInt.DistanceTo(p1) + pInt.DistanceTo(p2)) / 2.0;
                    cir = new Circle3D(new Frame3D(pInt, daxe, d), rad);
                    double alpha1 = global::Kerf.Engine.Geometry.ElCLib.Parameter(cir, p1);
                    double alpha3 = global::Kerf.Engine.Geometry.ElCLib.Parameter(cir, p2);
                    CircleCurve3D circ = new CircleCurve3D(cir);
                    _theArc = new TrimmedCurve3D(circ, alpha1, alpha3, true);
                }
            }
        }
    }

    /// <summary>The arc of Circ between the parameters of P1 and P2, in the circle's sense if Sense is true.</summary>
    public MakeArcOfCircle(in Circle3D circ, in Point3D p1, in Point3D p2, bool sense)
    {
        double alpha1 = global::Kerf.Engine.Geometry.ElCLib.Parameter(circ, p1);
        double alpha2 = global::Kerf.Engine.Geometry.ElCLib.Parameter(circ, p2);
        CircleCurve3D c = new CircleCurve3D(circ);
        _theArc = new TrimmedCurve3D(c, alpha1, alpha2, sense);
        _theError = Done;
    }

    /// <summary>The arc of Circ from the parameter of P to the parameter Alpha.</summary>
    public MakeArcOfCircle(in Circle3D circ, in Point3D p, double alpha, bool sense)
    {
        double alphafirst = global::Kerf.Engine.Geometry.ElCLib.Parameter(circ, p);
        CircleCurve3D c = new CircleCurve3D(circ);
        _theArc = new TrimmedCurve3D(c, alphafirst, alpha, sense);
        _theError = Done;
    }

    /// <summary>The arc of Circ between the parameters Alpha1 and Alpha2.</summary>
    public MakeArcOfCircle(in Circle3D circ, double alpha1, double alpha2, bool sense)
    {
        CircleCurve3D c = new CircleCurve3D(circ);
        _theArc = new TrimmedCurve3D(c, alpha1, alpha2, sense);
        _theError = Done;
    }

    /// <summary>The constructed arc (frozen).</summary>
    /// <exception cref="NotDoneException">No arc was constructed.</exception>
    public TrimmedCurve3D Value()
    {
        StdFail_NotDone_Raise_if(_theError != Done, "GC_MakeArcOfCircle::Value() - no result");
        _theArc!.Freeze();
        return _theArc;
    }

    /// <summary>The constructed arc (OCCT's implicit conversion).</summary>
    public static explicit operator TrimmedCurve3D(MakeArcOfCircle maker) => maker.Value();
}
