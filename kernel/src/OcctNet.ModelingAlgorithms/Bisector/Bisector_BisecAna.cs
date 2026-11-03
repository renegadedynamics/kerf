// Port of: src/ModelingAlgorithms/TKTopAlgo/Bisector/Bisector_BisecAna.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/Bisector/Bisector_BisecAna.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Geometry;
using static Occt.GccInt_IType;
using static Kerf.Geometry.JoinKind;

namespace Occt;

/// <summary>
/// The bisecting curve of two elements that are circles, lines or points: a conic (line, circle, ellipse, parabola or
/// hyperbola, from GccAna's bisectors) trimmed at the point it starts from.
/// </summary>
/// <remarks>
/// A geometry class: its mutators (Perform, Init, SetTrim, Reverse, Transform) are internal and check that the
/// curve isn't frozen. It owns its trimmed conic; a frozen one (given to Init) is copied before it changes.
/// OCCT's "Not yet implemented" messages to std::cout, for arguments that aren't circles, lines or points, are left
/// out.
/// </remarks>
public sealed class Bisector_BisecAna : Bisector_Curve
{
    private Geom2d_TrimmedCurve thebisector = null!;

    /// <summary>An empty bisector: Perform or Init must be called before it is used.</summary>
    public Bisector_BisecAna()
    {
    }

    // OCCT.NET: the trimmed conic may have been given frozen to Init; copy it before changing it.
    private void ownBisector()
    {
        if (thebisector.IsFrozen)
        {
            thebisector = (Geom2d_TrimmedCurve)thebisector.Copy();
        }
    }

    //=============================================================================
    //              calcul the distance between the point and the bissectrice.
    //              and orientation of the bissectrice.
    //    apoint        :       point of passage.
    //    abisector     :       calculated bissectrice.
    //    afirstvector  :       first vector. \
    //    asecondvector :       second vector./ to choose the proper sector.
    //    adirection    :       shows if the bissectrice is interior or exterior.
    //    aparameter    : out : the start parameter of the bissectrice.
    //    asense        : out : the direction of the bissectrice.
    //    astatus       : out : shows if the bissectrice is preserved.
    //=============================================================================
    private double Distance(in Point2D apoint, GccInt_Bisec abisector, in Vector2D afirstvector,
                            in Vector2D asecondvector, in Vector2D VecRef, double adirection, ref double aparameter,
                            ref bool asense, out bool astatus, bool IsBisecOfTwoLines = false)
    {
        astatus = true;

        Hyperbola2D gphyperbola;
        Parabola2D gpparabola;
        Ellipse2D gpellipse;
        Circle2D gpcircle;
        Line2D gpline;

        double distance = 0.0;
        Vector2D tangent = new();
        Point2D point = new();

        GccInt_IType type = abisector.ArcType();

        if (type == GccInt_Lin)
        {
            gpline = abisector.Line();
            aparameter = global::Kerf.Engine.Geometry.ElCLib.Parameter(gpline, apoint);
            global::Kerf.Engine.Geometry.ElCLib.D1(aparameter, gpline, out point, out tangent);
        }
        else if (type == GccInt_Cir)
        {
            gpcircle = abisector.Circle();
            aparameter = global::Kerf.Engine.Geometry.ElCLib.Parameter(gpcircle, apoint);
            global::Kerf.Engine.Geometry.ElCLib.D1(aparameter, gpcircle, out point, out tangent);
        }
        else if (type == GccInt_Hpr)
        {
            gphyperbola = abisector.Hyperbola();
            aparameter = global::Kerf.Engine.Geometry.ElCLib.Parameter(gphyperbola, apoint);
            global::Kerf.Engine.Geometry.ElCLib.D1(aparameter, gphyperbola, out point, out tangent);
        }
        else if (type == GccInt_Par)
        {
            gpparabola = abisector.Parabola();
            aparameter = global::Kerf.Engine.Geometry.ElCLib.Parameter(gpparabola, apoint);
            global::Kerf.Engine.Geometry.ElCLib.D1(aparameter, gpparabola, out point, out tangent);
        }
        else if (type == GccInt_Ell)
        {
            gpellipse = abisector.Ellipse();
            aparameter = global::Kerf.Engine.Geometry.ElCLib.Parameter(gpellipse, apoint);
            global::Kerf.Engine.Geometry.ElCLib.D1(aparameter, gpellipse, out point, out tangent);
        }

        distance = apoint.Distance(point);

        Direction2D afirstdir = new(afirstvector);
        Direction2D aseconddir = new(asecondvector);
        Direction2D tangdir = new(tangent);
        Direction2D secdirrev = aseconddir.Reversed();

        // 1st passage to learn if the curve is in the proper sector

        if (asense)
        {
            // the status is determined only in case on curve ie:
            // tangent to the bissectrice is bisectrice of two vectors.
            double SinPlat = 1.0e-3;
            if (Math.Abs(afirstdir ^ aseconddir) < SinPlat)
            { // flat
                if (afirstdir * aseconddir >= 0.0)
                { // tangent mixed
                    // correct if the scalar product is close to 1.
                    if (Math.Abs(tangdir * afirstdir) < 0.5)
                    {
                        astatus = false;
                    }
                }
                else
                { // opposed tangents.
                    // correct if the scalar product is close to 0.
                    if (Math.Abs(tangdir * afirstdir) > 0.5)
                    {
                        astatus = false;
                    }
                }
            }
            else if ((afirstdir ^ tangdir) * (tangdir ^ aseconddir) < -1.0E-8)
            {
                astatus = false;
            }
        }
        else
        {
            asense = true;

            if (!IsBisecOfTwoLines)
            {
                //  Modified by Sergey KHROMOV - Tue Oct 22 16:35:51 2002 Begin
                // Replacement of -1.E-8 for a tolerance 1.e-4
                double aTol = 1.0e-4;

                if ((afirstdir ^ secdirrev) * adirection < -0.1)
                { // input
                    if ((afirstdir ^ tangdir) * adirection < aTol && (secdirrev ^ tangdir) * adirection < aTol)
                    {
                        asense = false;
                    }
                }
                else if ((afirstdir ^ secdirrev) * adirection > 0.1)
                { // output
                    if ((afirstdir ^ tangdir) * adirection < aTol || (secdirrev ^ tangdir) * adirection < aTol)
                    {
                        asense = false;
                    }
                }
                else
                { // flat
                    if (afirstdir.Dot(secdirrev) > 0.0)
                    { // tangent
                        if ((afirstdir ^ tangdir) * adirection < 0.0)
                        {
                            asense = false;
                        }
                    }
                    else
                    { // turn back
                        //  Modified by Sergey KHROMOV - Thu Oct 31 14:16:53 2002
                        //  if ((afirstdir.Dot(tangdir))*adirection > 0.) asense = false;
                        if (afirstdir.Dot(tangdir) < 0.0)
                        {
                            asense = false;
                        }
                        //  Modified by Sergey KHROMOV - Thu Oct 31 14:16:54 2002
                    }
                }

                // jgv: for OCC26185
                if (VecRef.SquareMagnitude() != 0)
                {
                    Direction2D DirRef = new(VecRef);
                    if (tangdir * DirRef < 0.0)
                    {
                        asense = false;
                    }
                }
                ///////////////////
                //  Modified by Sergey KHROMOV - Tue Oct 22 16:35:51 2002 End
            }
        }
        return distance;
    }

    //===========================================================================
    //    calculate the bissectrice between two curves coming from a point.
    //
    //   afirstcurve   : \ curves the bissectrice between which will be calculated.
    //   asecondcurve  : /
    //   apoint        :   point through which the bissectrice should pass.
    //   afirstvector  : \ vectors to find the sector where
    //   asecondvector : / the bissectrice should be located.
    //   adirection    :   shows the side of the bissectrice to be preserved.
    //   tolerance     :   threshold starting from which the bisectrices are degenerated
    //===========================================================================
    /// <summary>
    /// Computes the bisecting curve of two lines or circles (possibly trimmed) from the point apoint, in the sector
    /// defined by afirstvector, asecondvector and adirection. oncurve is true if apoint is common to the curves.
    /// </summary>
    /// <exception cref="NotDoneException">A curve is neither a line nor a circle.</exception>
    internal void Perform(Geom2d_Curve afirstcurve, Geom2d_Curve asecondcurve, in Point2D apoint,
                          in Vector2D afirstvector, in Vector2D asecondvector, double adirection,
                          JoinKind ajointype, double tolerance, bool oncurve = true)
    {
        CheckMutable();
        bool ok;
        double distanceptsol, parameter = 0.0, firstparameter = 0.0;
        bool thesense = false, sense;
        double distancemini;
        int nbsolution;
        double PreConf = global::Kerf.Engine.Foundation.Precision.Confusion();

        Type type1 = afirstcurve.GetType();
        Type type2 = asecondcurve.GetType();
        Geom2d_Curve CurveF;
        Geom2d_Curve CurveE;
        GccInt_Bisec? TheSol = null;

        // jgv: for OCC26296
        Vector2D LineBisVec = new(0.0, 0.0);
        afirstcurve.D1(afirstcurve.LastParameter(), out Point2D Pnt1, out Vector2D tan1);
        asecondcurve.D1(asecondcurve.FirstParameter(), out Point2D Pnt2, out Vector2D tan2);
        if (!oncurve)
        {
            LineBisVec = new Vector2D(Pnt1, Pnt2);
            LineBisVec.Rotate(M_PI / 2.0);
        }
        ///////////////////

        tan1.Reverse();

        if (type1 == typeof(Geom2d_TrimmedCurve))
        {
            CurveF = ((Geom2d_TrimmedCurve)afirstcurve).ChangeBasisCurve();
        }
        else
        {
            CurveF = afirstcurve;
        }

        if (type2 == typeof(Geom2d_TrimmedCurve))
        {
            CurveE = ((Geom2d_TrimmedCurve)asecondcurve).ChangeBasisCurve();
        }
        else
        {
            CurveE = asecondcurve;
        }

        type1 = CurveF.GetType();
        type2 = CurveE.GetType();
        int cas = 0;
        Circle2D circle1 = new(), circle2 = new();
        Line2D line1 = new(), line2 = new();

        //=============================================================================
        //                Determination of the nature of arguments.
        //=============================================================================

        if (type1 == typeof(Geom2d_Circle))
        {
            if (type2 == typeof(Geom2d_Circle))
            {
                cas = 1;
                Geom2d_Circle C1 = (Geom2d_Circle)CurveF;
                circle1 = C1.Circ2d();
                Geom2d_Circle C2 = (Geom2d_Circle)CurveE;
                circle2 = C2.Circ2d();
            }
            else if (type2 == typeof(Geom2d_Line))
            {
                cas = 2;
                Geom2d_Circle C1 = (Geom2d_Circle)CurveF;
                circle1 = C1.Circ2d();
                Geom2d_Line L2 = (Geom2d_Line)CurveE;
                line2 = L2.Lin2d();
            }
            // else: OCCT prints "Not yet implemented" (left out).
        }
        else if (type1 == typeof(Geom2d_Line))
        {
            if (type2 == typeof(Geom2d_Circle))
            {
                cas = 2;
                Geom2d_Circle C1 = (Geom2d_Circle)CurveE;
                circle1 = C1.Circ2d();
                Geom2d_Line L2 = (Geom2d_Line)CurveF;
                line2 = L2.Lin2d();
            }
            else if (type2 == typeof(Geom2d_Line))
            {
                cas = 3;
                Geom2d_Line L1 = (Geom2d_Line)CurveF;
                line1 = L1.Lin2d();
                Geom2d_Line L2 = (Geom2d_Line)CurveE;
                line2 = L2.Lin2d();
            }
            // else: OCCT prints "Not yet implemented" (left out).
        }
        // else: OCCT prints "Not yet implemented" (left out).

        switch (cas)
        {
            //=============================================================================
            //                       Bissectrice circle - circle.
            //=============================================================================
            case 1:
            {
                double radius1 = circle1.Radius();
                double radius2 = circle2.Radius();

                //-----------------------------------------------------
                // Particular case when two circles are mixed.
                //-----------------------------------------------------
                if (circle1.Location().IsEqual(circle2.Location(), PreConf)
                    && (Math.Abs(radius1 - radius2) <= PreConf))
                {
                    Point2D P1 = afirstcurve.Value(afirstcurve.LastParameter());
                    Point2D P2 = asecondcurve.Value(asecondcurve.FirstParameter());
                    Point2D PMil;
                    Line2D line;
                    PMil = new Point2D((P1.X() + P2.X()) / 2.0, (P1.Y() + P2.Y()) / 2.0);
                    //  Modified by skv - Fri Jul  1 16:23:32 2005 IDEM(Airbus) Begin
                    if (!circle1.Location().IsEqual(PMil, PreConf))
                    {
                        // PMil doesn't coincide with the circle location.
                        line = new Line2D(PMil,
                                            new Direction2D(circle1.Location().X() - PMil.X(),
                                                         circle1.Location().Y() - PMil.Y()));
                    }
                    else if (radius1 >= PreConf)
                    {
                        // PMil coincides with the circle location and radius is greater then 0.
                        line = new Line2D(circle1.Location(),
                                            new Direction2D(P1.Y() - circle1.Location().Y(),
                                                         circle1.Location().X() - P1.X()));
                    }
                    else
                    {
                        // radius is equal to 0. No matter what direction to chose.
                        line = new Line2D(circle1.Location(), new Direction2D(Direction2D.D.X));
                    }
                    //  Modified by skv - Fri Jul  1 16:23:32 2005 IDEM(Airbus) End

                    GccInt_Bisec solution = new GccInt_BLine(line);
                    sense = false;
                    //  Modified by skv - Tue Feb 15 17:51:29 2005 Integration Begin
                    if (oncurve)
                    {
                        distanceptsol = Distance(apoint, solution, tan2, tan1, LineBisVec, adirection, ref parameter,
                                                 ref sense, out ok);
                    }
                    else
                    {
                        distanceptsol = Distance(apoint, solution, afirstvector, asecondvector, LineBisVec, adirection,
                                                 ref parameter, ref sense, out ok);
                    }
                    //  Modified by skv - Tue Feb 15 17:51:29 2005 Integration End
                    Geom2d_Curve bisectorcurve = new Geom2d_Line(line);
                    if (!sense)
                    {
                        thebisector = new Geom2d_TrimmedCurve(bisectorcurve, parameter, -global::Kerf.Engine.Foundation.Precision.Infinite());
                    }
                    else
                    {
                        double parameter2;
                        parameter2 = global::Kerf.Engine.Geometry.ElCLib.Parameter(line, circle1.Location());
                        parameter2 += 1.0e-8;
                        thebisector = new Geom2d_TrimmedCurve(bisectorcurve, parameter, parameter2);
                    }
                    break;
                } // end of case mixed circles.

                if (radius1 < radius2)
                {
                    Circle2D circle = circle1;
                    circle1 = circle2;
                    circle2 = circle;

                    double radius = radius1;
                    radius1 = radius2;
                    radius2 = radius;
                }

                // small reframing of circles. in the case when the circles
                // are OnCurve , if they are almost tangent they become tangent.
                double EntreAxe = circle1.Location().Distance(circle2.Location());
                double D1 = 0.5 * (radius1 - EntreAxe - radius2);
                bool CirclesTangent = false;

                //  Modified by Sergey KHROMOV - Thu Oct 31 12:42:21 2002 End
                if (oncurve && Math.Abs(D1) < PreConf && tan1.IsParallel(tan2, 1.0e-8))
                {
                    //  Modified by Sergey KHROMOV - Thu Oct 31 12:42:22 2002 Begin
                    // C2 included in C1 and tangent.
                    circle1.SetRadius(radius1 - D1);
                    circle2.SetRadius(radius2 + D1);
                    CirclesTangent = true;
                }
                else
                {
                    D1 = 0.5 * (radius1 - EntreAxe + radius2);
                    //  Modified by Sergey KHROMOV - Thu Oct 31 12:44:24 2002 Begin
                    if (oncurve && Math.Abs(D1) < PreConf && tan1.IsParallel(tan2, 1.0e-8))
                    {
                        //  Modified by Sergey KHROMOV - Thu Oct 31 12:44:25 2002 End
                        // C2 and C1 tangent and disconnected.
                        circle1.SetRadius(radius1 - D1);
                        circle2.SetRadius(radius2 - D1);
                        CirclesTangent = true;
                    }
                } // end of reframing.

                GccAna_Circ2dBisec Bisector = new(circle1, circle2);

                distancemini = global::Kerf.Engine.Foundation.Precision.Infinite();

                if (Bisector.IsDone())
                {
                    nbsolution = Bisector.NbSolutions();
                    for (int i = 1; i <= nbsolution; i++)
                    {
                        GccInt_Bisec solution = Bisector.ThisSolution(i);
                        Degenerate(ref solution, tolerance);
                        sense = true;
                        if (oncurve)
                        {
                            distanceptsol = Distance(apoint, solution, tan1, tan2, LineBisVec, adirection,
                                                     ref parameter, ref sense, out ok);
                        }
                        else
                        {
                            ok = true;
                        }

                        if (ok)
                        {
                            sense = false;
                            //  Modified by skv - Tue Feb 15 17:51:29 2005 Integration Begin
                            if (oncurve)
                            {
                                distanceptsol = Distance(apoint, solution, tan2, tan1, LineBisVec, adirection,
                                                         ref parameter, ref sense, out ok);
                            }
                            else
                            {
                                distanceptsol = Distance(apoint, solution, afirstvector, asecondvector, LineBisVec,
                                                         adirection, ref parameter, ref sense, out ok);
                            }
                            //  Modified by skv - Tue Feb 15 17:51:29 2005 Integration End
                            if (distanceptsol <= distancemini)
                            {
                                TheSol = solution;
                                firstparameter = parameter;
                                thesense = sense;
                                distancemini = distanceptsol;
                            }
                        }
                    }
                    // OCCT: with no solution kept, the bisector stays unset (occt-issues.md).
                    if (TheSol is not null)
                    {
                        Geom2d_Curve bisectorcurve;
                        GccInt_IType type = TheSol.ArcType();
                        if (type == GccInt_Lin)
                        {
                            Line2D gpline = TheSol.Line();
                            bisectorcurve = new Geom2d_Line(gpline);

                            double secondparameter = global::Kerf.Engine.Foundation.Precision.Infinite();
                            if (!thesense)
                            {
                                secondparameter = -global::Kerf.Engine.Foundation.Precision.Infinite();
                            }

                            if (oncurve)
                            {
                                // bisectrice right and oncurve
                                // is cut between two circle of the same radius if circles are tangent.

                                // if tangent flat and the bissectrice at the side of the concavity
                                // of one of the circles. the bissectrice is a segment of the point common to
                                // first of 2 centers of circle that it meets.
                                // in this case it is important to set a segmnent for
                                // intersection in Tool2d.

                                if (CirclesTangent)
                                {
                                    //  Modified by skv - Tue Apr 13 17:23:31 2004 IDEM(Airbus) Begin
                                    //  Trying to correct the line if the distance between it
                                    //  and the reference point is too big.
                                    if (distancemini > tolerance)
                                    {
                                        Point2D aPloc = gpline.Location();
                                        Direction2D aNewDir = new(apoint.XY() - aPloc.XY());
                                        Line2D aNewLin = new(aPloc, aNewDir);
                                        Point2D aCC2 = circle2.Location();
                                        double aNewDMin = aNewLin.Distance(apoint);
                                        double aTolConf = 1.0e-3;
                                        // Hope, aNewDMin is equal to 0...

                                        if (aNewLin.Distance(aCC2) <= aTolConf)
                                        {
                                            distancemini = aNewDMin;
                                            firstparameter = global::Kerf.Engine.Geometry.ElCLib.Parameter(aNewLin, apoint);
                                            bisectorcurve = new Geom2d_Line(aNewLin);
                                        }
                                    }
                                    //  Modified by skv - Tue Apr 13 17:23:32 2004 IDEM(Airbus) End
                                    if (tan1.Dot(tan2) < 0.0)
                                    {
                                        // flat and not turn back.
                                        double Par1 = global::Kerf.Engine.Geometry.ElCLib.Parameter(gpline, circle1.Location());
                                        double Par2 = global::Kerf.Engine.Geometry.ElCLib.Parameter(gpline, circle2.Location());
                                        double MinPar = Min(Par1, Par2);
                                        double MaxPar = Max(Par1, Par2);

                                        if (!thesense)
                                        {
                                            if (MaxPar < firstparameter)
                                            {
                                                secondparameter = MaxPar - 1.0E-8;
                                            }
                                            else if (MinPar < firstparameter)
                                            {
                                                secondparameter = MinPar - 1.0E-8;
                                            }
                                        }
                                        else
                                        {
                                            if (MinPar > firstparameter)
                                            {
                                                secondparameter = MinPar + 1.0E-8;
                                            }
                                            else if (MaxPar > firstparameter)
                                            {
                                                secondparameter = MaxPar + 1.0E-8;
                                            }
                                        }
                                    }
                                }
                            }

                            thebisector = new Geom2d_TrimmedCurve(bisectorcurve, firstparameter, secondparameter);
                        }
                        else if (type == GccInt_Cir)
                        {
                            bisectorcurve = new Geom2d_Circle(TheSol.Circle());
                            if (!thesense)
                            {
                                thebisector = new Geom2d_TrimmedCurve(bisectorcurve, firstparameter - 2.0 * M_PI,
                                                                      firstparameter, thesense);
                            }
                            else
                            {
                                thebisector = new Geom2d_TrimmedCurve(bisectorcurve, firstparameter,
                                                                      firstparameter + 2.0 * M_PI, thesense);
                            }
                        }
                        else if (type == GccInt_Hpr)
                        {
                            bisectorcurve = new Geom2d_Hyperbola(TheSol.Hyperbola());
                            if (!thesense)
                            {
                                thebisector =
                                    new Geom2d_TrimmedCurve(bisectorcurve, firstparameter, -global::Kerf.Engine.Foundation.Precision.Infinite());
                            }
                            else
                            {
                                thebisector =
                                    new Geom2d_TrimmedCurve(bisectorcurve, firstparameter, global::Kerf.Engine.Foundation.Precision.Infinite());
                            }
                        }
                        else if (type == GccInt_Ell)
                        {
                            bisectorcurve = new Geom2d_Ellipse(TheSol.Ellipse());
                            if (!thesense)
                            {
                                thebisector = new Geom2d_TrimmedCurve(bisectorcurve, firstparameter - 2.0 * M_PI,
                                                                      firstparameter, thesense);
                            }
                            else
                            {
                                thebisector = new Geom2d_TrimmedCurve(bisectorcurve, firstparameter,
                                                                      firstparameter + 2.0 * M_PI, thesense);
                            }
                        }
                    }
                }
            }
            break;

            //=============================================================================
            //                       Bissectrice circle - straight.
            //=============================================================================
            case 2:
            {
                // small reframing of circles. in case OnCurve.
                // If the circle and the straight line are almost tangent they become tangent.
                if (oncurve)
                {
                    double radius1 = circle1.Radius();
                    double D1 = (line2.Distance(circle1.Location()) - radius1);
                    //  Modified by Sergey KHROMOV - Wed Oct 30 14:48:43 2002 Begin
                    if (Math.Abs(D1) < PreConf && tan1.IsParallel(tan2, 1.0e-8))
                    {
                        //  Modified by Sergey KHROMOV - Wed Oct 30 14:48:44 2002 End
                        circle1.SetRadius(radius1 + D1);
                    }
                }

                GccAna_CircLin2dBisec Bisector = new(circle1, line2);

                distancemini = global::Kerf.Engine.Foundation.Precision.Infinite();

                if (Bisector.IsDone())
                {
                    nbsolution = Bisector.NbSolutions();
                    for (int i = 1; i <= nbsolution; i++)
                    {
                        GccInt_Bisec solution = Bisector.ThisSolution(i);
                        Degenerate(ref solution, tolerance);
                        sense = true;
                        distanceptsol = Distance(apoint, solution, tan1, tan2, LineBisVec, adirection,
                                                 ref parameter, ref sense, out ok);
                        if (ok || !oncurve)
                        {
                            sense = false;
                            //  Modified by skv - Tue Feb 15 17:51:29 2005 Integration Begin
                            if (oncurve)
                            {
                                distanceptsol = Distance(apoint, solution, tan2, tan1, LineBisVec, adirection,
                                                         ref parameter, ref sense, out ok);
                            }
                            else
                            {
                                distanceptsol = Distance(apoint, solution, afirstvector, asecondvector, LineBisVec,
                                                         adirection, ref parameter, ref sense, out ok);
                            }
                            //  Modified by skv - Tue Feb 15 17:51:29 2005 Integration End
                            if (distanceptsol <= distancemini)
                            {
                                TheSol = solution;
                                firstparameter = parameter;
                                thesense = sense;
                                distancemini = distanceptsol + 1.0e-8;
                            }
                        }
                    }
                    // OCCT: with no solution kept, the bisector stays unset (occt-issues.md).
                    if (TheSol is not null)
                    {
                        GccInt_IType type = TheSol.ArcType();
                        Geom2d_Curve bisectorcurve;
                        if (type == GccInt_Lin)
                        {
                            // -----------------------------------------------------------------
                            // If the bisectrice is a line
                            //       => the straight line is tangent to the circle.
                            //       It the part of bisectrice concerned is at the side of the center.
                            //       => the bisectrice is limited by the point and the center of the circle.
                            // Note : In the latter case the bisectrice is a degenerated parabole.
                            // -----------------------------------------------------------------
                            Point2D circlecenter;
                            Line2D gpline;
                            double secondparameter;

                            circlecenter = circle1.Location();
                            gpline = TheSol.Line();
                            secondparameter = global::Kerf.Engine.Geometry.ElCLib.Parameter(gpline, circlecenter);
                            bisectorcurve = new Geom2d_Line(gpline);

                            if (!thesense)
                            {
                                if (secondparameter > firstparameter)
                                {
                                    secondparameter = -global::Kerf.Engine.Foundation.Precision.Infinite();
                                }
                                else
                                {
                                    secondparameter = secondparameter - 1.0E-8;
                                }
                            }
                            else
                            {
                                if (secondparameter < firstparameter)
                                {
                                    secondparameter = global::Kerf.Engine.Foundation.Precision.Infinite();
                                }
                                else
                                {
                                    secondparameter = secondparameter + 1.0E-8;
                                }
                            }

                            thebisector = new Geom2d_TrimmedCurve(bisectorcurve, firstparameter, secondparameter);
                        }
                        else if (type == GccInt_Par)
                        {
                            bisectorcurve = new Geom2d_Parabola(TheSol.Parabola());
                            Point2D apex = bisectorcurve.Value(0.0);
                            Point2D firstpnt = bisectorcurve.Value(firstparameter);
                            double ChordLen = apex.Distance(firstpnt);
                            double TolPar = 1.0e-5;
                            double secondparameter = global::Kerf.Engine.Foundation.Precision.Infinite();
                            if (!thesense)
                            {
                                if (ajointype == Intersection && TolPar < firstparameter
                                    && ChordLen >= circle1.Radius())
                                { // first parameter is too far from peak of parabola
                                    secondparameter = 0.0;
                                }
                                thebisector = new Geom2d_TrimmedCurve(bisectorcurve, firstparameter, -secondparameter);
                            }
                            else
                            {
                                if (ajointype == Intersection && firstparameter < -TolPar
                                    && ChordLen >= circle1.Radius())
                                { // first parameter is too far from peak of parabola
                                    secondparameter = 0.0;
                                }
                                thebisector = new Geom2d_TrimmedCurve(bisectorcurve, firstparameter, secondparameter);
                            }
                        }
                    }
                }
            }
            break;

            //=============================================================================
            //                       Bissectrice straight - straight.
            //=============================================================================
            case 3:
            {
                Direction2D Direc1 = line1.Direction();
                Direction2D Direc2 = line2.Direction();
                Line2D line;
                distancemini = global::Kerf.Engine.Foundation.Precision.Infinite();

                //  Modified by Sergey KHROMOV - Tue Sep 10 15:58:43 2002 Begin
                //  Change to the same criterion as in MAT2d_Circuit.cxx:
                //         method MAT2d_Circuit::InitOpen(..)
                if (Direc1.IsParallel(Direc2, 1.0e-8))
                {
                    //  Modified by Sergey KHROMOV - Tue Sep 10 15:58:45 2002 End
                    if (line1.Distance(line2.Location()) / 2.0 <= global::Kerf.Engine.Foundation.Precision.Confusion())
                    {
                        line = new Line2D(apoint, new Direction2D(-line1.Direction().Y(), line1.Direction().X()));
                    }
                    else
                    {
                        line = new Line2D(apoint, line2.Direction());
                    }

                    GccInt_Bisec solution = new GccInt_BLine(line);
                    //  Modified by skv - Wed Jul  7 17:21:09 2004 IDEM(Airbus) Begin
                    sense = false;
                    //  Modified by skv - Tue Feb 15 17:51:29 2005 Integration Begin
                    if (oncurve)
                    {
                        distanceptsol = Distance(apoint, solution, tan2, tan1, LineBisVec, adirection, ref parameter,
                                                 ref sense, out ok);
                    }
                    else
                    {
                        distanceptsol = Distance(apoint, solution, afirstvector, asecondvector, LineBisVec, adirection,
                                                 ref parameter, ref sense, out ok);
                    }
                    //  Modified by skv - Tue Feb 15 17:51:29 2005 Integration End
                    firstparameter = parameter;
                    Geom2d_Curve bisectorcurve;
                    bisectorcurve = new Geom2d_Line(line);
                    if (!sense)
                    {
                        thebisector = new Geom2d_TrimmedCurve(bisectorcurve, firstparameter, -global::Kerf.Engine.Foundation.Precision.Infinite());
                    }
                    else
                    {
                        thebisector = new Geom2d_TrimmedCurve(bisectorcurve, firstparameter, global::Kerf.Engine.Foundation.Precision.Infinite());
                    }
                    //  Modified by skv - Wed Jul  7 17:21:09 2004 IDEM(Airbus) End
                }
                else
                {
                    Line2D l = new(apoint, new Direction2D(Direc2.XY() - Direc1.XY()));
                    GccInt_Bisec solution = new GccInt_BLine(l);
                    bool isOk;
                    sense = false;
                    //  Modified by skv - Tue Feb 15 17:51:29 2005 Integration Begin
                    if (oncurve)
                    {
                        distanceptsol = Distance(apoint, solution, tan2, tan1, LineBisVec, adirection, ref parameter,
                                                 ref sense, out isOk);
                    }
                    else
                    {
                        distanceptsol = Distance(apoint, solution, afirstvector, asecondvector, LineBisVec, adirection,
                                                 ref parameter, ref sense, out isOk, true);
                    }
                    //  Modified by skv - Tue Feb 15 17:51:29 2005 Integration End
                    if (isOk || !oncurve)
                    {
                        thesense = sense;
                        distancemini = distanceptsol;
                    }
                    TheSol = new GccInt_BLine(l);
                    Geom2d_Curve bisectorcurve;
                    bisectorcurve = new Geom2d_Line(TheSol.Line());
                    if (!thesense)
                    {
                        thebisector = new Geom2d_TrimmedCurve(bisectorcurve, 0.0, -global::Kerf.Engine.Foundation.Precision.Infinite());
                    }
                    else
                    {
                        thebisector = new Geom2d_TrimmedCurve(bisectorcurve, 0.0, global::Kerf.Engine.Foundation.Precision.Infinite());
                    }
                }
            }
            break;

            default:
                throw new NotDoneException();
        }
    }

    //===========================================================================
    //  calculate the bissectrice between a curve and a point and starting in a point.
    //
    //   afirstcurve   : \ curve and point the bissectrice between which is calculated
    //   asecondpoint  : /
    //   apoint        :   point through which the bissectrice should pass.
    //   afirstvector  : \ vectors to determine the sector in which
    //   asecondvector : / the bissectrice should be located.
    //   adirection    :   shows the side of the bissectrice to be preserved.
    //   tolerance     :   threshold starting from which the bisectrices are degenerated
    //===========================================================================
    /// <summary>
    /// Computes the bisecting curve of a line or circle (possibly trimmed) and a point, from the point apoint.
    /// oncurve is true if apoint is the point asecondpoint.
    /// </summary>
    internal void Perform(Geom2d_Curve afirstcurve, Geom2d_Point asecondpoint, in Point2D apoint,
                          in Vector2D afirstvector, in Vector2D asecondvector, double adirection, double tolerance,
                          bool oncurve = true)
    {
        CheckMutable();
        bool ok;
        bool thesense = false, sense;
        double distanceptsol, parameter = 0.0, firstparameter = 0.0, secondparameter;
        Vector2D VecRef = new(0.0, 0.0);
        Geom2d_Curve curve;
        GccInt_Bisec? TheSol = null;

        Circle2D circle = new();
        Line2D line = new();
        Point2D circlecenter;

        int cas = 0;

        Type aFirstCurveType = afirstcurve.GetType();

        if (aFirstCurveType == typeof(Geom2d_TrimmedCurve))
        {
            curve = ((Geom2d_TrimmedCurve)afirstcurve).ChangeBasisCurve();
        }
        else
        {
            curve = afirstcurve;
        }

        aFirstCurveType = curve.GetType();
        // OCCT reads asecondpoint->Pnt2d() into an unused constant here.

        if (aFirstCurveType == typeof(Geom2d_Circle))
        {
            cas = 1;
            Geom2d_Circle C1 = (Geom2d_Circle)curve;
            circle = C1.Circ2d();
        }
        else if (aFirstCurveType == typeof(Geom2d_Line))
        {
            cas = 2;
            Geom2d_Line L1 = (Geom2d_Line)curve;
            line = L1.Lin2d();
        }
        // else: OCCT prints "Not yet implemented" (left out).

        switch (cas)
        {
            //=============================================================================
            //                       Bissectrice point - circle.
            //=============================================================================
            case 1:
            {
                GccAna_CircPnt2dBisec Bisector = new(circle, asecondpoint.Pnt2d(), tolerance);
                double distancemini = global::Kerf.Engine.Foundation.Precision.Infinite();
                if (Bisector.IsDone())
                {
                    int nbsolution = Bisector.NbSolutions();
                    for (int i = 1; i <= nbsolution; i++)
                    {
                        GccInt_Bisec solution = Bisector.ThisSolution(i);
                        Degenerate(ref solution, tolerance);
                        sense = false;
                        distanceptsol = Distance(apoint, solution, afirstvector, asecondvector, VecRef, adirection,
                                                 ref parameter, ref sense, out ok);

                        if (distanceptsol <= distancemini)
                        {
                            TheSol = solution;
                            firstparameter = parameter;
                            thesense = sense;
                            distancemini = distanceptsol;
                        }
                    }
                    // OCCT: with no solution kept, the bisector stays unset (occt-issues.md).
                    if (TheSol is not null)
                    {
                        GccInt_IType aSolType = TheSol.ArcType();
                        Geom2d_Curve bisectorcurve;
                        if (aSolType == GccInt_Lin)
                        {
                            // ----------------------------------------------------------------------------
                            // If the bisectrice is a line
                            //       => the point is on the circle.
                            //       If the part of bisectrice concerned is at the side of the center.
                            //       => the bisectrice is limited by the point and the center of the circle.
                            // Note : In this latter case the bisectrice is actually an ellipse of small null axis.
                            // ----------------------------------------------------------------------------

                            circlecenter = circle.Location();
                            line = TheSol.Line();
                            secondparameter = global::Kerf.Engine.Geometry.ElCLib.Parameter(line, circlecenter);
                            bisectorcurve = new Geom2d_Line(line);

                            if (!thesense)
                            {
                                if (secondparameter > firstparameter)
                                {
                                    secondparameter = -global::Kerf.Engine.Foundation.Precision.Infinite();
                                }
                                else
                                {
                                    secondparameter = secondparameter - 1.0E-8;
                                }
                            }
                            else
                            {
                                if (secondparameter < firstparameter)
                                {
                                    secondparameter = global::Kerf.Engine.Foundation.Precision.Infinite();
                                }
                                else
                                {
                                    secondparameter = secondparameter + 1.0E-8;
                                }
                            }

                            thebisector = new Geom2d_TrimmedCurve(bisectorcurve, firstparameter, secondparameter);
                        }
                        else if (aSolType == GccInt_Cir)
                        {
                            bisectorcurve = new Geom2d_Circle(TheSol.Circle());
                            if (!thesense)
                            {
                                thebisector = new Geom2d_TrimmedCurve(bisectorcurve, firstparameter - 2.0 * M_PI,
                                                                      firstparameter, thesense);
                            }
                            else
                            {
                                thebisector = new Geom2d_TrimmedCurve(bisectorcurve, firstparameter,
                                                                      firstparameter + 2.0 * M_PI, thesense);
                            }
                        }
                        else if (aSolType == GccInt_Hpr)
                        {
                            bisectorcurve = new Geom2d_Hyperbola(TheSol.Hyperbola());
                            if (!thesense)
                            {
                                thebisector =
                                    new Geom2d_TrimmedCurve(bisectorcurve, firstparameter, -global::Kerf.Engine.Foundation.Precision.Infinite());
                            }
                            else
                            {
                                thebisector =
                                    new Geom2d_TrimmedCurve(bisectorcurve, firstparameter, global::Kerf.Engine.Foundation.Precision.Infinite());
                            }
                        }
                        else if (aSolType == GccInt_Ell)
                        {
                            bisectorcurve = new Geom2d_Ellipse(TheSol.Ellipse());
                            if (!thesense)
                            {
                                thebisector = new Geom2d_TrimmedCurve(bisectorcurve, firstparameter - 2.0 * M_PI,
                                                                      firstparameter, thesense);
                            }
                            else
                            {
                                thebisector = new Geom2d_TrimmedCurve(bisectorcurve, firstparameter,
                                                                      firstparameter + 2.0 * M_PI, thesense);
                            }
                        }
                    }
                }
            }
            break;

            //=============================================================================
            //                       Bissectrice point - straight.
            //=============================================================================
            case 2:
            {
                GccAna_LinPnt2dBisec Bisector = new(line, asecondpoint.Pnt2d());

                GccInt_Bisec solution = Bisector.ThisSolution();
                Degenerate(ref solution, tolerance);
                GccInt_IType type = solution.ArcType();
                Geom2d_Curve bisectorcurve = null!;

                if (type == GccInt_Lin)
                {
                    bisectorcurve = new Geom2d_Line(solution.Line());
                }
                else if (type == GccInt_Par)
                {
                    bisectorcurve = new Geom2d_Parabola(solution.Parabola());
                }
                sense = false;
                distanceptsol = Distance(apoint, solution, afirstvector, asecondvector, VecRef, adirection,
                                         ref parameter, ref sense, out ok);
                if (ok || !oncurve)
                {
                    firstparameter = parameter;
                    thesense = sense;
                }

                if (!thesense)
                {
                    thebisector = new Geom2d_TrimmedCurve(bisectorcurve, firstparameter, -global::Kerf.Engine.Foundation.Precision.Infinite());
                }
                else
                {
                    thebisector = new Geom2d_TrimmedCurve(bisectorcurve, firstparameter, global::Kerf.Engine.Foundation.Precision.Infinite());
                }
            }
            break;

            default:
                // OCCT prints "Not yet implemented" (left out).
                break;
        }
    }

    //===========================================================================
    //  calculate the bissectrice between a curve and a point starting at a point.
    //
    //   afirstpoint   : \ curves between which the
    //   asecondcurve  : / bissectrice is calculated.
    //   apoint        :   point through which the bissectrice should pass.
    //   afirstvector  : \ vectors to determine the secteur in which
    //   asecondvector : / the bissectrice should be located.
    //   adirection    :   shows the side of the bissectrice to be preserved.
    //   tolerance     :   threshold at which the bisectrices become degenerated
    //===========================================================================
    /// <summary>
    /// Computes the bisecting curve of a point and a line or circle, from the point apoint: the curve-point bisector
    /// with the vectors exchanged and the direction reversed, and a tolerance of 0 (Tolerance is unused, as in
    /// OCCT).
    /// </summary>
    internal void Perform(Geom2d_Point afirstpoint, Geom2d_Curve asecondcurve, in Point2D apoint,
                          in Vector2D afirstvector, in Vector2D asecondvector, double adirection, double Tolerance,
                          bool oncurve = true)
    {
        CheckMutable();
        double adirectionreverse = -adirection;
        Perform(asecondcurve, afirstpoint, apoint, asecondvector, afirstvector, adirectionreverse,
                //  Modified by skv - Tue Feb 15 17:51:29 2005 Integration Begin
                0.0,
                //  Modified by skv - Tue Feb 15 17:51:29 2005 Integration End
                oncurve);
    }

    //===========================================================================
    //        calculate the bissectrice between two points starting at a point.
    //
    //   afirstpoint   : \ curves between which the
    //   asecondpoint  : / bissectrice is calculated.
    //   apoint        :   point through which the bissectrice should pass.
    //   afirstvector  : \ vectors to determine the sector in which the
    //   asecondvector : / bissectrice should be located.
    //   adirection    :   shows the side of the bissectrice to be preserved.
    //===========================================================================
    /// <summary>
    /// Computes the bisecting line of two points, from the point apoint (Tolerance is unused, as in OCCT).
    /// </summary>
    /// <exception cref="GccAna_NoSolution">The points coincide.</exception>
    internal void Perform(Geom2d_Point afirstpoint, Geom2d_Point asecondpoint, in Point2D apoint,
                          in Vector2D afirstvector, in Vector2D asecondvector, double adirection,
                          double Tolerance = 0.0, bool oncurve = true)
    {
        CheckMutable();
        bool sense, ok;
        double parameter = 0.0;
        Vector2D VecRef = new(0.0, 0.0);

        GccAna_Pnt2dBisec bisector = new(afirstpoint.Pnt2d(), asecondpoint.Pnt2d());
        Line2D line = bisector.ThisSolution();
        GccInt_Bisec solution = new GccInt_BLine(line);

        sense = false;
        Distance(apoint, solution, afirstvector, asecondvector, VecRef, adirection, ref parameter, ref sense, out ok);
        // OCCT: otherwise the bisector stays unset, and Bisector_Bisec dereferences it (occt-issues.md).
        if (ok || !oncurve)
        {
            Geom2d_Curve bisectorcurve = new Geom2d_Line(line);
            if (!sense)
            {
                thebisector = new Geom2d_TrimmedCurve(bisectorcurve, parameter, -global::Kerf.Engine.Foundation.Precision.Infinite());
            }
            else
            {
                thebisector = new Geom2d_TrimmedCurve(bisectorcurve, parameter, global::Kerf.Engine.Foundation.Precision.Infinite());
            }
        }
    }

    /// <summary>False: an analytic bisector is never extended.</summary>
    public override bool IsExtendAtStart() => false;

    /// <summary>False: an analytic bisector is never extended.</summary>
    public override bool IsExtendAtEnd() => false;

    //=============================================================================
    //           The domain of the curve is the set of points that are closer to the
    //           than to its extremities.
    //           For the calculation the domain is extended. Extension of Epsilon1 of the
    //           First and the Last parameter of the curve.
    //=============================================================================
    /// <summary>
    /// Trims the bisector by the domain of the curve Cu: does nothing, OCCT's body is commented out.
    /// </summary>
    internal void SetTrim(Geom2d_Curve Cu)
    {
        CheckMutable();
        // OCCT's body is commented out (the trimming by the limit lines of Cu's domain).
    }

    /// <summary>Trims the bisector to [uf, ul].</summary>
    internal void SetTrim(double uf, double ul)
    {
        CheckMutable();
        ownBisector();
        thebisector.SetTrim(uf, ul);
    }

    /// <summary>Reverses the bisector.</summary>
    protected internal override void Reverse()
    {
        CheckMutable();
        ownBisector();
        thebisector.Reverse();
    }

    /// <summary>The trimmed conic's reversed parameter.</summary>
    public override double ReversedParameter(double U) => thebisector.ReversedParameter(U);

    /// <summary>The trimmed conic's IsCN.</summary>
    /// <exception cref="RangeErrorException">N &lt; 0.</exception>
    public override bool IsCN(int N) => thebisector.IsCN(N);

    /// <summary>A new, unfrozen bisector with a copy of the trimmed conic.</summary>
    public override Geom2d_Geometry Copy()
    {
        Bisector_BisecAna C = new();
        C.Init((Geom2d_TrimmedCurve)thebisector.Copy());
        return C;
    }

    /// <summary>Transforms the trimmed conic.</summary>
    protected internal override void Transform(in Transform2D T)
    {
        CheckMutable();
        ownBisector();
        thebisector.Transform(T);
    }

    /// <summary>The first parameter of the trimmed conic.</summary>
    public override double FirstParameter()
    {
        //  modified by NIZHNY-EAP Thu Feb  3 17:23:42 2000 ___BEGIN___
        //  return thebisector->BasisCurve()->FirstParameter();
        return thebisector.FirstParameter();
        //  modified by NIZHNY-EAP Thu Feb  3 17:23:48 2000 ___END___
    }

    /// <summary>The last parameter of the trimmed conic.</summary>
    public override double LastParameter() => thebisector.LastParameter();

    /// <summary>True if the conic is closed.</summary>
    public override bool IsClosed() => thebisector.ChangeBasisCurve().IsClosed();

    /// <summary>True if the conic is periodic.</summary>
    public override bool IsPeriodic() => thebisector.ChangeBasisCurve().IsPeriodic();

    /// <summary>The trimmed conic's continuity.</summary>
    public override Continuity Continuity() => thebisector.Continuity();

    /// <summary>The conic's point at U (not limited to the trimmed range).</summary>
    public override Point2D EvalD0(double U) => thebisector.ChangeBasisCurve().EvalD0(U);

    /// <summary>The conic's point and first derivative at U.</summary>
    public override ResD1 EvalD1(double U) => thebisector.ChangeBasisCurve().EvalD1(U);

    /// <summary>The conic's point and first two derivatives at U.</summary>
    public override ResD2 EvalD2(double U) => thebisector.ChangeBasisCurve().EvalD2(U);

    /// <summary>The conic's point and first three derivatives at U.</summary>
    public override ResD3 EvalD3(double U) => thebisector.ChangeBasisCurve().EvalD3(U);

    /// <summary>The conic's derivative of order N at U.</summary>
    public override Vector2D EvalDN(double U, int N) => thebisector.ChangeBasisCurve().EvalDN(U, N);

    /// <summary>The conic (line, circle, ellipse, parabola or hyperbola), frozen.</summary>
    public Geom2d_Curve Geom2dCurve() => thebisector.BasisCurve();

    /// <summary>The parameter of the point P on the conic.</summary>
    public override double Parameter(in Point2D P)
    {
        Hyperbola2D gphyperbola;
        Parabola2D gpparabola;
        Ellipse2D gpellipse;
        Circle2D gpcircle;
        Line2D gpline;

        Geom2d_Curve BasisCurve = thebisector.ChangeBasisCurve();
        Type Type = BasisCurve.GetType();

        if (Type == typeof(Geom2d_Line))
        {
            gpline = ((Geom2d_Line)BasisCurve).Lin2d();
            return global::Kerf.Engine.Geometry.ElCLib.Parameter(gpline, P);
        }
        else if (Type == typeof(Geom2d_Circle))
        {
            gpcircle = ((Geom2d_Circle)BasisCurve).Circ2d();
            return global::Kerf.Engine.Geometry.ElCLib.Parameter(gpcircle, P);
        }
        else if (Type == typeof(Geom2d_Hyperbola))
        {
            gphyperbola = ((Geom2d_Hyperbola)BasisCurve).Hypr2d();
            return global::Kerf.Engine.Geometry.ElCLib.Parameter(gphyperbola, P);
        }
        else if (Type == typeof(Geom2d_Parabola))
        {
            gpparabola = ((Geom2d_Parabola)BasisCurve).Parab2d();
            return global::Kerf.Engine.Geometry.ElCLib.Parameter(gpparabola, P);
        }
        else if (Type == typeof(Geom2d_Ellipse))
        {
            gpellipse = ((Geom2d_Ellipse)BasisCurve).Elips2d();
            return global::Kerf.Engine.Geometry.ElCLib.Parameter(gpellipse, P);
        }
        return 0.0;
    }

    /// <summary>The parameter of the start point: the first parameter of the trimmed conic.</summary>
    public double ParameterOfStartPoint() => thebisector.FirstParameter();

    /// <summary>The parameter of the end point: the last parameter of the trimmed conic.</summary>
    public double ParameterOfEndPoint() => thebisector.LastParameter();

    /// <summary>1: an analytic bisector is one interval.</summary>
    public override int NbIntervals() => 1;

    /// <summary>The first parameter.</summary>
    /// <exception cref="OutOfRangeException">I is not 1.</exception>
    public override double IntervalFirst(int I)
    {
        if (I != 1)
        {
            throw new OutOfRangeException();
        }
        return FirstParameter();
    }

    /// <summary>The last parameter.</summary>
    /// <exception cref="OutOfRangeException">I is not 1.</exception>
    public override double IntervalLast(int I)
    {
        if (I != 1)
        {
            throw new OutOfRangeException();
        }
        return LastParameter();
    }

    /// <summary>Makes Bis the bisector (shared, as OCCT's handle; copied before this curve changes if frozen).</summary>
    internal void Init(Geom2d_TrimmedCurve Bis)
    {
        CheckMutable();
        thebisector = Bis;
    }

    //=============================================================================
    //           if the bisectrice is an ellipse, a parabole or a degenerated ellipse.
    //=============================================================================
    private static bool Degenerate(ref GccInt_Bisec aBisector, double Tolerance)
    {
        bool Degeneree = false;

        Hyperbola2D gphyperbola;
        Parabola2D gpparabola;
        Ellipse2D gpellipse;

        GccInt_Bisec NewBisector;

        GccInt_IType type = aBisector.ArcType();

        if (type == GccInt_Hpr)
        {
            gphyperbola = aBisector.Hyperbola();

            // If the Hyperbola is degenerated, it is replaced by the straight line
            // with direction to the axis if symmetry.

            if (gphyperbola.MajorRadius() < Tolerance)
            {
                Line2D gpline = new(gphyperbola.YAxis());
                NewBisector = new GccInt_BLine(gpline);
                aBisector = NewBisector;
                Degeneree = true;
            }
            if (gphyperbola.MinorRadius() < Tolerance)
            {
                Line2D gpline = new(gphyperbola.XAxis());
                NewBisector = new GccInt_BLine(gpline);
                aBisector = NewBisector;
                Degeneree = true;
            }
        }
        else if (type == GccInt_Par)
        {
            gpparabola = aBisector.Parabola();

            // If the parabole is degenerated, it is replaces by the straight
            // line starting at the Top and with direction of the axis of symmetry.

            if (gpparabola.Focal() < Tolerance)
            {
                Line2D gpline = new(gpparabola.MirrorAxis());
                NewBisector = new GccInt_BLine(gpline);
                aBisector = NewBisector;
                Degeneree = true;
            }
        }
        else if (type == GccInt_Ell)
        {
            gpellipse = aBisector.Ellipse();

            // If the ellipse is degenerated, it is replaced by the straight line
            // defined by the great axis.

            if (gpellipse.MinorRadius() < Tolerance)
            {
                Line2D gpline = new(gpellipse.XAxis());
                NewBisector = new GccInt_BLine(gpline);
                aBisector = NewBisector;
                Degeneree = true;
            }
        }
        return Degeneree;
    }

    // OCCT's Dump (std::cout) is left out.
}
