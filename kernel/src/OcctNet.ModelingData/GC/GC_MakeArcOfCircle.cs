// Port of: src/ModelingData/TKGeomBase/GC/GC_MakeArcOfCircle.hxx
// Port of: src/ModelingData/TKGeomBase/GC/GC_MakeArcOfCircle.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Geometry;
using static Occt.gce_ErrorType;

namespace Occt;

/// <summary>
/// Constructions of an arc of circle in 3D as a Geom_TrimmedCurve: through three points, from two points and the
/// tangent at the first, or on a circle between two points or parameters.
/// </summary>
public sealed class GC_MakeArcOfCircle : GC_Root
{
    private Geom_TrimmedCurve? TheArc;

    /// <summary>The arc through P1, P2 and P3, from P1 to P3; fails with the status of gce_MakeCirc(P1, P2, P3).</summary>
    public GC_MakeArcOfCircle(in Point3D P1, in Point3D P2, in Point3D P3)
    {
        bool sense;
        //
        gce_MakeCirc Cir = new gce_MakeCirc(P1, P2, P3);
        TheError = Cir.Status();
        if (TheError == gce_Done)
        {
            double Alpha1, Alpha3; //,Alpha2
            Circle3D C = Cir.Value();
            // modified by NIZNHY-PKV Thu Mar  3 10:53:02 2005: Alpha1 is always 0 (OCCT computed the three
            // parameters and the sense before).
            Alpha1 = 0.0;
            Alpha3 = global::Kerf.Engine.Geometry.ElCLib.Parameter(C, P3);
            sense = true;

            Geom_Circle Circ = new Geom_Circle(C);
            TheArc = new Geom_TrimmedCurve(Circ, Alpha1, Alpha3, sense);
        }
    }

    /// <summary>
    /// The arc from P1 to P2 tangent to V at P1. Fails with gce_ConfusedPoints if P1 and P2 coincide, and with
    /// gce_IntersectionError if the lines defining the center don't intersect.
    /// </summary>
    public GC_MakeArcOfCircle(in Point3D P1, in Vector3D V, in Point3D P2)
    {
        Circle3D cir;
        gce_MakeLin Corde = new gce_MakeLin(P1, P2);
        TheError = Corde.Status();
        if (TheError == gce_Done)
        {
            Line3D corde = Corde.Value();
            Direction3D dir = corde.Direction();
            Direction3D dbid = new Direction3D(V);
            Direction3D Daxe = dbid ^ dir;
            Direction3D Dir1 = Daxe ^ dir;
            Line3D bis = new Line3D(
                new Point3D((P1.X() + P2.X()) / 2.0, (P1.Y() + P2.Y()) / 2.0, (P1.Z() + P2.Z()) / 2.0), Dir1);
            Direction3D d = dbid ^ Daxe;
            Line3D norm = new Line3D(P1, d);
            Extrema_ExtElC distmin = new Extrema_ExtElC(bis, norm, global::Kerf.Engine.Foundation.Precision.Confusion());
            if (!distmin.IsDone())
            {
                TheError = gce_IntersectionError;
            }
            else
            {
                int nbext = distmin.NbExt();
                if (nbext == 0)
                {
                    TheError = gce_IntersectionError;
                }
                else
                {
                    double TheDist = RealLast();
                    Point3D pInt = new(), pon1, pon2;
                    int i = 1;
                    while (i <= nbext)
                    {
                        if (distmin.SquareDistance(i) < TheDist)
                        {
                            TheDist = distmin.SquareDistance(i);
                            distmin.Points(i, out Extrema_POnCurv Pon1, out Extrema_POnCurv Pon2);
                            pon1 = Pon1.Value();
                            pon2 = Pon2.Value();
                            pInt = new Point3D((pon1.XYZ() + pon2.XYZ()) / 2.0);
                        }
                        i++;
                    }
                    double Rad = (pInt.Distance(P1) + pInt.Distance(P2)) / 2.0;
                    cir = new Circle3D(new Frame3D(pInt, Daxe, d), Rad);
                    double Alpha1 = global::Kerf.Engine.Geometry.ElCLib.Parameter(cir, P1);
                    double Alpha3 = global::Kerf.Engine.Geometry.ElCLib.Parameter(cir, P2);
                    Geom_Circle Circ = new Geom_Circle(cir);
                    TheArc = new Geom_TrimmedCurve(Circ, Alpha1, Alpha3, true);
                }
            }
        }
    }

    /// <summary>The arc of Circ between the parameters of P1 and P2, in the circle's sense if Sense is true.</summary>
    public GC_MakeArcOfCircle(in Circle3D Circ, in Point3D P1, in Point3D P2, bool Sense)
    {
        double Alpha1 = global::Kerf.Engine.Geometry.ElCLib.Parameter(Circ, P1);
        double Alpha2 = global::Kerf.Engine.Geometry.ElCLib.Parameter(Circ, P2);
        Geom_Circle C = new Geom_Circle(Circ);
        TheArc = new Geom_TrimmedCurve(C, Alpha1, Alpha2, Sense);
        TheError = gce_Done;
    }

    /// <summary>The arc of Circ from the parameter of P to the parameter Alpha.</summary>
    public GC_MakeArcOfCircle(in Circle3D Circ, in Point3D P, double Alpha, bool Sense)
    {
        double Alphafirst = global::Kerf.Engine.Geometry.ElCLib.Parameter(Circ, P);
        Geom_Circle C = new Geom_Circle(Circ);
        TheArc = new Geom_TrimmedCurve(C, Alphafirst, Alpha, Sense);
        TheError = gce_Done;
    }

    /// <summary>The arc of Circ between the parameters Alpha1 and Alpha2.</summary>
    public GC_MakeArcOfCircle(in Circle3D Circ, double Alpha1, double Alpha2, bool Sense)
    {
        Geom_Circle C = new Geom_Circle(Circ);
        TheArc = new Geom_TrimmedCurve(C, Alpha1, Alpha2, Sense);
        TheError = gce_Done;
    }

    /// <summary>The constructed arc (frozen).</summary>
    /// <exception cref="NotDoneException">No arc was constructed.</exception>
    public Geom_TrimmedCurve Value()
    {
        StdFail_NotDone_Raise_if(TheError != gce_Done, "GC_MakeArcOfCircle::Value() - no result");
        TheArc!.Freeze();
        return TheArc;
    }

    /// <summary>The constructed arc (OCCT's implicit conversion).</summary>
    public static explicit operator Geom_TrimmedCurve(GC_MakeArcOfCircle theMaker) => theMaker.Value();
}
