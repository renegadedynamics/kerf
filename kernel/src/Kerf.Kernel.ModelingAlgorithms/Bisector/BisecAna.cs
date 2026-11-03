// Port of: src/ModelingAlgorithms/TKTopAlgo/Bisector/Bisector_BisecAna.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/Bisector/Bisector_BisecAna.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Constraints;
using Kerf.Engine.Construction.Constraints.Analytic;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Objects;
using Kerf.Geometry;
using Occt;
using static Kerf.Engine.Construction.Constraints.ConstraintCurveType;
using static Kerf.Geometry.JoinKind;

namespace Kerf.Engine.Offsets.Bisectors;

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
public sealed class BisecAna : BisectorCurve
{
    private TrimmedCurve2D _thebisector = null!;

    /// <summary>An empty bisector: Perform or Init must be called before it is used.</summary>
    public BisecAna()
    {
    }

    // OCCT.NET: the trimmed conic may have been given frozen to Init; copy it before changing it.
    private void OwnBisector()
    {
        if (_thebisector.IsFrozen)
        {
            _thebisector = (TrimmedCurve2D)_thebisector.Copy();
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
    private double Distance(in Point2D apoint, ConstraintBisec abisector, in Vector2D afirstvector,
                            in Vector2D asecondvector, in Vector2D vecRef, double adirection, ref double aparameter,
                            ref bool asense, out bool astatus, bool isBisecOfTwoLines = false)
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

        ConstraintCurveType type = abisector.ArcType();

        if (type == ConstraintCurveType.Lin)
        {
            gpline = abisector.Line();
            aparameter = global::Kerf.Engine.Geometry.ElCLib.Parameter(gpline, apoint);
            global::Kerf.Engine.Geometry.ElCLib.D1(aparameter, gpline, out point, out tangent);
        }
        else if (type == ConstraintCurveType.Cir)
        {
            gpcircle = abisector.Circle();
            aparameter = global::Kerf.Engine.Geometry.ElCLib.Parameter(gpcircle, apoint);
            global::Kerf.Engine.Geometry.ElCLib.D1(aparameter, gpcircle, out point, out tangent);
        }
        else if (type == ConstraintCurveType.Hpr)
        {
            gphyperbola = abisector.Hyperbola();
            aparameter = global::Kerf.Engine.Geometry.ElCLib.Parameter(gphyperbola, apoint);
            global::Kerf.Engine.Geometry.ElCLib.D1(aparameter, gphyperbola, out point, out tangent);
        }
        else if (type == ConstraintCurveType.Par)
        {
            gpparabola = abisector.Parabola();
            aparameter = global::Kerf.Engine.Geometry.ElCLib.Parameter(gpparabola, apoint);
            global::Kerf.Engine.Geometry.ElCLib.D1(aparameter, gpparabola, out point, out tangent);
        }
        else if (type == ConstraintCurveType.Ell)
        {
            gpellipse = abisector.Ellipse();
            aparameter = global::Kerf.Engine.Geometry.ElCLib.Parameter(gpellipse, apoint);
            global::Kerf.Engine.Geometry.ElCLib.D1(aparameter, gpellipse, out point, out tangent);
        }

        distance = apoint.DistanceTo(point);

        Direction2D afirstdir = new(afirstvector);
        Direction2D aseconddir = new(asecondvector);
        Direction2D tangdir = new(tangent);
        Direction2D secdirrev = aseconddir.Reversed();

        // 1st passage to learn if the curve is in the proper sector

        if (asense)
        {
            // the status is determined only in case on curve ie:
            // tangent to the bissectrice is bisectrice of two vectors.
            double sinPlat = 1.0e-3;
            if (Math.Abs(afirstdir ^ aseconddir) < sinPlat)
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

            if (!isBisecOfTwoLines)
            {
                //  Modified by Sergey KHROMOV - Tue Oct 22 16:35:51 2002 Begin
                // Replacement of -1.E-8 for a tolerance 1.e-4
                double tol = 1.0e-4;

                if ((afirstdir ^ secdirrev) * adirection < -0.1)
                { // input
                    if ((afirstdir ^ tangdir) * adirection < tol && (secdirrev ^ tangdir) * adirection < tol)
                    {
                        asense = false;
                    }
                }
                else if ((afirstdir ^ secdirrev) * adirection > 0.1)
                { // output
                    if ((afirstdir ^ tangdir) * adirection < tol || (secdirrev ^ tangdir) * adirection < tol)
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
                if (vecRef.SquareMagnitude != 0)
                {
                    Direction2D dirRef = new(vecRef);
                    if (tangdir * dirRef < 0.0)
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
    internal void Perform(Curve2D afirstcurve, Curve2D asecondcurve, in Point2D apoint,
                          in Vector2D afirstvector, in Vector2D asecondvector, double adirection,
                          JoinKind ajointype, double tolerance, bool oncurve = true)
    {
        CheckMutable();
        bool ok;
        double distanceptsol, parameter = 0.0, firstparameter = 0.0;
        bool thesense = false, sense;
        double distancemini;
        int nbsolution;
        double preConf = global::Kerf.Engine.Foundation.Precision.Confusion();

        Type type1 = afirstcurve.GetType();
        Type type2 = asecondcurve.GetType();
        Curve2D curveF;
        Curve2D curveE;
        ConstraintBisec? theSol = null;

        // jgv: for OCC26296
        Vector2D lineBisVec = new(0.0, 0.0);
        afirstcurve.D1(afirstcurve.LastParameter, out Point2D pnt1, out Vector2D tan1);
        asecondcurve.D1(asecondcurve.FirstParameter, out Point2D pnt2, out Vector2D tan2);
        if (!oncurve)
        {
            lineBisVec = new Vector2D(pnt1, pnt2);
            lineBisVec = lineBisVec.Rotated(M_PI / 2.0);
        }
        ///////////////////

        tan1 = tan1.Reversed();

        if (type1 == typeof(TrimmedCurve2D))
        {
            curveF = ((TrimmedCurve2D)afirstcurve).ChangeBasisCurve();
        }
        else
        {
            curveF = afirstcurve;
        }

        if (type2 == typeof(TrimmedCurve2D))
        {
            curveE = ((TrimmedCurve2D)asecondcurve).ChangeBasisCurve();
        }
        else
        {
            curveE = asecondcurve;
        }

        type1 = curveF.GetType();
        type2 = curveE.GetType();
        int cas = 0;
        Circle2D circle1 = new(), circle2 = new();
        Line2D line1 = new(), line2 = new();

        //=============================================================================
        //                Determination of the nature of arguments.
        //=============================================================================

        if (type1 == typeof(CircleCurve2D))
        {
            if (type2 == typeof(CircleCurve2D))
            {
                cas = 1;
                CircleCurve2D c1 = (CircleCurve2D)curveF;
                circle1 = c1.Circle;
                CircleCurve2D c2 = (CircleCurve2D)curveE;
                circle2 = c2.Circle;
            }
            else if (type2 == typeof(LineCurve2D))
            {
                cas = 2;
                CircleCurve2D c1 = (CircleCurve2D)curveF;
                circle1 = c1.Circle;
                LineCurve2D l2 = (LineCurve2D)curveE;
                line2 = l2.Line;
            }
            // else: OCCT prints "Not yet implemented" (left out).
        }
        else if (type1 == typeof(LineCurve2D))
        {
            if (type2 == typeof(CircleCurve2D))
            {
                cas = 2;
                CircleCurve2D c1 = (CircleCurve2D)curveE;
                circle1 = c1.Circle;
                LineCurve2D l2 = (LineCurve2D)curveF;
                line2 = l2.Line;
            }
            else if (type2 == typeof(LineCurve2D))
            {
                cas = 3;
                LineCurve2D l1 = (LineCurve2D)curveF;
                line1 = l1.Line;
                LineCurve2D l2 = (LineCurve2D)curveE;
                line2 = l2.Line;
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
                double radius1 = circle1.Radius;
                double radius2 = circle2.Radius;

                //-----------------------------------------------------
                // Particular case when two circles are mixed.
                //-----------------------------------------------------
                if (circle1.Location.IsEqual(circle2.Location, preConf)
                    && (Math.Abs(radius1 - radius2) <= preConf))
                {
                    Point2D p1 = afirstcurve.Value(afirstcurve.LastParameter);
                    Point2D p2 = asecondcurve.Value(asecondcurve.FirstParameter);
                    Point2D pMil;
                    Line2D line;
                    pMil = new Point2D((p1.X + p2.X) / 2.0, (p1.Y + p2.Y) / 2.0);
                    //  Modified by skv - Fri Jul  1 16:23:32 2005 IDEM(Airbus) Begin
                    if (!circle1.Location.IsEqual(pMil, preConf))
                    {
                        // PMil doesn't coincide with the circle location.
                        line = new Line2D(pMil,
                                            new Direction2D(circle1.Location.X - pMil.X,
                                                         circle1.Location.Y - pMil.Y));
                    }
                    else if (radius1 >= preConf)
                    {
                        // PMil coincides with the circle location and radius is greater then 0.
                        line = new Line2D(circle1.Location,
                                            new Direction2D(p1.Y - circle1.Location.Y,
                                                         circle1.Location.X - p1.X));
                    }
                    else
                    {
                        // radius is equal to 0. No matter what direction to chose.
                        line = new Line2D(circle1.Location, new Direction2D(Direction2D.D.X));
                    }
                    //  Modified by skv - Fri Jul  1 16:23:32 2005 IDEM(Airbus) End

                    ConstraintBisec solution = new BLine(line);
                    sense = false;
                    //  Modified by skv - Tue Feb 15 17:51:29 2005 Integration Begin
                    if (oncurve)
                    {
                        distanceptsol = Distance(apoint, solution, tan2, tan1, lineBisVec, adirection, ref parameter,
                                                 ref sense, out ok);
                    }
                    else
                    {
                        distanceptsol = Distance(apoint, solution, afirstvector, asecondvector, lineBisVec, adirection,
                                                 ref parameter, ref sense, out ok);
                    }
                    //  Modified by skv - Tue Feb 15 17:51:29 2005 Integration End
                    Curve2D bisectorcurve = new LineCurve2D(line);
                    if (!sense)
                    {
                        _thebisector = new TrimmedCurve2D(bisectorcurve, parameter, -global::Kerf.Engine.Foundation.Precision.Infinite());
                    }
                    else
                    {
                        double parameter2;
                        parameter2 = global::Kerf.Engine.Geometry.ElCLib.Parameter(line, circle1.Location);
                        parameter2 += 1.0e-8;
                        _thebisector = new TrimmedCurve2D(bisectorcurve, parameter, parameter2);
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
                double entreAxe = circle1.Location.DistanceTo(circle2.Location);
                double d1 = 0.5 * (radius1 - entreAxe - radius2);
                bool circlesTangent = false;

                //  Modified by Sergey KHROMOV - Thu Oct 31 12:42:21 2002 End
                if (oncurve && Math.Abs(d1) < preConf && tan1.IsParallel(tan2, 1.0e-8))
                {
                    //  Modified by Sergey KHROMOV - Thu Oct 31 12:42:22 2002 Begin
                    // C2 included in C1 and tangent.
                    circle1 = circle1.WithRadius(radius1 - d1);
                    circle2 = circle2.WithRadius(radius2 + d1);
                    circlesTangent = true;
                }
                else
                {
                    d1 = 0.5 * (radius1 - entreAxe + radius2);
                    //  Modified by Sergey KHROMOV - Thu Oct 31 12:44:24 2002 Begin
                    if (oncurve && Math.Abs(d1) < preConf && tan1.IsParallel(tan2, 1.0e-8))
                    {
                        //  Modified by Sergey KHROMOV - Thu Oct 31 12:44:25 2002 End
                        // C2 and C1 tangent and disconnected.
                        circle1 = circle1.WithRadius(radius1 - d1);
                        circle2 = circle2.WithRadius(radius2 - d1);
                        circlesTangent = true;
                    }
                } // end of reframing.

                Circ2dBisec bisector = new(circle1, circle2);

                distancemini = global::Kerf.Engine.Foundation.Precision.Infinite();

                if (bisector.IsDone)
                {
                    nbsolution = bisector.SolutionCount;
                    for (int i = 1; i <= nbsolution; i++)
                    {
                        ConstraintBisec solution = bisector.ThisSolution(i);
                        Degenerate(ref solution, tolerance);
                        sense = true;
                        if (oncurve)
                        {
                            distanceptsol = Distance(apoint, solution, tan1, tan2, lineBisVec, adirection,
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
                                distanceptsol = Distance(apoint, solution, tan2, tan1, lineBisVec, adirection,
                                                         ref parameter, ref sense, out ok);
                            }
                            else
                            {
                                distanceptsol = Distance(apoint, solution, afirstvector, asecondvector, lineBisVec,
                                                         adirection, ref parameter, ref sense, out ok);
                            }
                            //  Modified by skv - Tue Feb 15 17:51:29 2005 Integration End
                            if (distanceptsol <= distancemini)
                            {
                                theSol = solution;
                                firstparameter = parameter;
                                thesense = sense;
                                distancemini = distanceptsol;
                            }
                        }
                    }
                    // OCCT: with no solution kept, the bisector stays unset (occt-issues.md).
                    if (theSol is not null)
                    {
                        Curve2D bisectorcurve;
                        ConstraintCurveType type = theSol.ArcType();
                        if (type == ConstraintCurveType.Lin)
                        {
                            Line2D gpline = theSol.Line();
                            bisectorcurve = new LineCurve2D(gpline);

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

                                if (circlesTangent)
                                {
                                    //  Modified by skv - Tue Apr 13 17:23:31 2004 IDEM(Airbus) Begin
                                    //  Trying to correct the line if the distance between it
                                    //  and the reference point is too big.
                                    if (distancemini > tolerance)
                                    {
                                        Point2D ploc = gpline.Location;
                                        Direction2D newDir = new(apoint.XY() - ploc.XY());
                                        Line2D newLin = new(ploc, newDir);
                                        Point2D cc2 = circle2.Location;
                                        double newDMin = newLin.DistanceTo(apoint);
                                        double tolConf = 1.0e-3;
                                        // Hope, aNewDMin is equal to 0...

                                        if (newLin.DistanceTo(cc2) <= tolConf)
                                        {
                                            distancemini = newDMin;
                                            firstparameter = global::Kerf.Engine.Geometry.ElCLib.Parameter(newLin, apoint);
                                            bisectorcurve = new LineCurve2D(newLin);
                                        }
                                    }
                                    //  Modified by skv - Tue Apr 13 17:23:32 2004 IDEM(Airbus) End
                                    if (tan1.Dot(tan2) < 0.0)
                                    {
                                        // flat and not turn back.
                                        double par1 = global::Kerf.Engine.Geometry.ElCLib.Parameter(gpline, circle1.Location);
                                        double par2 = global::Kerf.Engine.Geometry.ElCLib.Parameter(gpline, circle2.Location);
                                        double minPar = Min(par1, par2);
                                        double maxPar = Max(par1, par2);

                                        if (!thesense)
                                        {
                                            if (maxPar < firstparameter)
                                            {
                                                secondparameter = maxPar - 1.0E-8;
                                            }
                                            else if (minPar < firstparameter)
                                            {
                                                secondparameter = minPar - 1.0E-8;
                                            }
                                        }
                                        else
                                        {
                                            if (minPar > firstparameter)
                                            {
                                                secondparameter = minPar + 1.0E-8;
                                            }
                                            else if (maxPar > firstparameter)
                                            {
                                                secondparameter = maxPar + 1.0E-8;
                                            }
                                        }
                                    }
                                }
                            }

                            _thebisector = new TrimmedCurve2D(bisectorcurve, firstparameter, secondparameter);
                        }
                        else if (type == ConstraintCurveType.Cir)
                        {
                            bisectorcurve = new CircleCurve2D(theSol.Circle());
                            if (!thesense)
                            {
                                _thebisector = new TrimmedCurve2D(bisectorcurve, firstparameter - 2.0 * M_PI,
                                                                      firstparameter, thesense);
                            }
                            else
                            {
                                _thebisector = new TrimmedCurve2D(bisectorcurve, firstparameter,
                                                                      firstparameter + 2.0 * M_PI, thesense);
                            }
                        }
                        else if (type == ConstraintCurveType.Hpr)
                        {
                            bisectorcurve = new HyperbolaCurve2D(theSol.Hyperbola());
                            if (!thesense)
                            {
                                _thebisector =
                                    new TrimmedCurve2D(bisectorcurve, firstparameter, -global::Kerf.Engine.Foundation.Precision.Infinite());
                            }
                            else
                            {
                                _thebisector =
                                    new TrimmedCurve2D(bisectorcurve, firstparameter, global::Kerf.Engine.Foundation.Precision.Infinite());
                            }
                        }
                        else if (type == ConstraintCurveType.Ell)
                        {
                            bisectorcurve = new EllipseCurve2D(theSol.Ellipse());
                            if (!thesense)
                            {
                                _thebisector = new TrimmedCurve2D(bisectorcurve, firstparameter - 2.0 * M_PI,
                                                                      firstparameter, thesense);
                            }
                            else
                            {
                                _thebisector = new TrimmedCurve2D(bisectorcurve, firstparameter,
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
                    double radius1 = circle1.Radius;
                    double d1 = (line2.DistanceTo(circle1.Location) - radius1);
                    //  Modified by Sergey KHROMOV - Wed Oct 30 14:48:43 2002 Begin
                    if (Math.Abs(d1) < preConf && tan1.IsParallel(tan2, 1.0e-8))
                    {
                        //  Modified by Sergey KHROMOV - Wed Oct 30 14:48:44 2002 End
                        circle1 = circle1.WithRadius(radius1 + d1);
                    }
                }

                CircLin2dBisec bisector = new(circle1, line2);

                distancemini = global::Kerf.Engine.Foundation.Precision.Infinite();

                if (bisector.IsDone)
                {
                    nbsolution = bisector.SolutionCount;
                    for (int i = 1; i <= nbsolution; i++)
                    {
                        ConstraintBisec solution = bisector.ThisSolution(i);
                        Degenerate(ref solution, tolerance);
                        sense = true;
                        distanceptsol = Distance(apoint, solution, tan1, tan2, lineBisVec, adirection,
                                                 ref parameter, ref sense, out ok);
                        if (ok || !oncurve)
                        {
                            sense = false;
                            //  Modified by skv - Tue Feb 15 17:51:29 2005 Integration Begin
                            if (oncurve)
                            {
                                distanceptsol = Distance(apoint, solution, tan2, tan1, lineBisVec, adirection,
                                                         ref parameter, ref sense, out ok);
                            }
                            else
                            {
                                distanceptsol = Distance(apoint, solution, afirstvector, asecondvector, lineBisVec,
                                                         adirection, ref parameter, ref sense, out ok);
                            }
                            //  Modified by skv - Tue Feb 15 17:51:29 2005 Integration End
                            if (distanceptsol <= distancemini)
                            {
                                theSol = solution;
                                firstparameter = parameter;
                                thesense = sense;
                                distancemini = distanceptsol + 1.0e-8;
                            }
                        }
                    }
                    // OCCT: with no solution kept, the bisector stays unset (occt-issues.md).
                    if (theSol is not null)
                    {
                        ConstraintCurveType type = theSol.ArcType();
                        Curve2D bisectorcurve;
                        if (type == ConstraintCurveType.Lin)
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

                            circlecenter = circle1.Location;
                            gpline = theSol.Line();
                            secondparameter = global::Kerf.Engine.Geometry.ElCLib.Parameter(gpline, circlecenter);
                            bisectorcurve = new LineCurve2D(gpline);

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

                            _thebisector = new TrimmedCurve2D(bisectorcurve, firstparameter, secondparameter);
                        }
                        else if (type == ConstraintCurveType.Par)
                        {
                            bisectorcurve = new ParabolaCurve2D(theSol.Parabola());
                            Point2D apex = bisectorcurve.Value(0.0);
                            Point2D firstpnt = bisectorcurve.Value(firstparameter);
                            double chordLen = apex.DistanceTo(firstpnt);
                            double tolPar = 1.0e-5;
                            double secondparameter = global::Kerf.Engine.Foundation.Precision.Infinite();
                            if (!thesense)
                            {
                                if (ajointype == global::Kerf.Geometry.JoinKind.Intersection && tolPar < firstparameter
                                    && chordLen >= circle1.Radius)
                                { // first parameter is too far from peak of parabola
                                    secondparameter = 0.0;
                                }
                                _thebisector = new TrimmedCurve2D(bisectorcurve, firstparameter, -secondparameter);
                            }
                            else
                            {
                                if (ajointype == global::Kerf.Geometry.JoinKind.Intersection && firstparameter < -tolPar
                                    && chordLen >= circle1.Radius)
                                { // first parameter is too far from peak of parabola
                                    secondparameter = 0.0;
                                }
                                _thebisector = new TrimmedCurve2D(bisectorcurve, firstparameter, secondparameter);
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
                Direction2D direc1 = line1.Direction;
                Direction2D direc2 = line2.Direction;
                Line2D line;
                distancemini = global::Kerf.Engine.Foundation.Precision.Infinite();

                //  Modified by Sergey KHROMOV - Tue Sep 10 15:58:43 2002 Begin
                //  Change to the same criterion as in MAT2d_Circuit.cxx:
                //         method MAT2d_Circuit::InitOpen(..)
                if (direc1.IsParallel(direc2, 1.0e-8))
                {
                    //  Modified by Sergey KHROMOV - Tue Sep 10 15:58:45 2002 End
                    if (line1.DistanceTo(line2.Location) / 2.0 <= global::Kerf.Engine.Foundation.Precision.Confusion())
                    {
                        line = new Line2D(apoint, new Direction2D(-line1.Direction.Y, line1.Direction.X));
                    }
                    else
                    {
                        line = new Line2D(apoint, line2.Direction);
                    }

                    ConstraintBisec solution = new BLine(line);
                    //  Modified by skv - Wed Jul  7 17:21:09 2004 IDEM(Airbus) Begin
                    sense = false;
                    //  Modified by skv - Tue Feb 15 17:51:29 2005 Integration Begin
                    if (oncurve)
                    {
                        distanceptsol = Distance(apoint, solution, tan2, tan1, lineBisVec, adirection, ref parameter,
                                                 ref sense, out ok);
                    }
                    else
                    {
                        distanceptsol = Distance(apoint, solution, afirstvector, asecondvector, lineBisVec, adirection,
                                                 ref parameter, ref sense, out ok);
                    }
                    //  Modified by skv - Tue Feb 15 17:51:29 2005 Integration End
                    firstparameter = parameter;
                    Curve2D bisectorcurve;
                    bisectorcurve = new LineCurve2D(line);
                    if (!sense)
                    {
                        _thebisector = new TrimmedCurve2D(bisectorcurve, firstparameter, -global::Kerf.Engine.Foundation.Precision.Infinite());
                    }
                    else
                    {
                        _thebisector = new TrimmedCurve2D(bisectorcurve, firstparameter, global::Kerf.Engine.Foundation.Precision.Infinite());
                    }
                    //  Modified by skv - Wed Jul  7 17:21:09 2004 IDEM(Airbus) End
                }
                else
                {
                    Line2D l = new(apoint, new Direction2D(direc2.XY() - direc1.XY()));
                    ConstraintBisec solution = new BLine(l);
                    bool isOk;
                    sense = false;
                    //  Modified by skv - Tue Feb 15 17:51:29 2005 Integration Begin
                    if (oncurve)
                    {
                        distanceptsol = Distance(apoint, solution, tan2, tan1, lineBisVec, adirection, ref parameter,
                                                 ref sense, out isOk);
                    }
                    else
                    {
                        distanceptsol = Distance(apoint, solution, afirstvector, asecondvector, lineBisVec, adirection,
                                                 ref parameter, ref sense, out isOk, true);
                    }
                    //  Modified by skv - Tue Feb 15 17:51:29 2005 Integration End
                    if (isOk || !oncurve)
                    {
                        thesense = sense;
                        distancemini = distanceptsol;
                    }
                    theSol = new BLine(l);
                    Curve2D bisectorcurve;
                    bisectorcurve = new LineCurve2D(theSol.Line());
                    if (!thesense)
                    {
                        _thebisector = new TrimmedCurve2D(bisectorcurve, 0.0, -global::Kerf.Engine.Foundation.Precision.Infinite());
                    }
                    else
                    {
                        _thebisector = new TrimmedCurve2D(bisectorcurve, 0.0, global::Kerf.Engine.Foundation.Precision.Infinite());
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
    internal void Perform(Curve2D afirstcurve, Point2 asecondpoint, in Point2D apoint,
                          in Vector2D afirstvector, in Vector2D asecondvector, double adirection, double tolerance,
                          bool oncurve = true)
    {
        CheckMutable();
        bool ok;
        bool thesense = false, sense;
        double distanceptsol, parameter = 0.0, firstparameter = 0.0, secondparameter;
        Vector2D vecRef = new(0.0, 0.0);
        Curve2D curve;
        ConstraintBisec? theSol = null;

        Circle2D circle = new();
        Line2D line = new();
        Point2D circlecenter;

        int cas = 0;

        Type firstCurveType = afirstcurve.GetType();

        if (firstCurveType == typeof(TrimmedCurve2D))
        {
            curve = ((TrimmedCurve2D)afirstcurve).ChangeBasisCurve();
        }
        else
        {
            curve = afirstcurve;
        }

        firstCurveType = curve.GetType();
        // OCCT reads asecondpoint->Pnt2d() into an unused constant here.

        if (firstCurveType == typeof(CircleCurve2D))
        {
            cas = 1;
            CircleCurve2D c1 = (CircleCurve2D)curve;
            circle = c1.Circle;
        }
        else if (firstCurveType == typeof(LineCurve2D))
        {
            cas = 2;
            LineCurve2D l1 = (LineCurve2D)curve;
            line = l1.Line;
        }
        // else: OCCT prints "Not yet implemented" (left out).

        switch (cas)
        {
            //=============================================================================
            //                       Bissectrice point - circle.
            //=============================================================================
            case 1:
            {
                CircPnt2dBisec bisector = new(circle, asecondpoint.Pnt2d(), tolerance);
                double distancemini = global::Kerf.Engine.Foundation.Precision.Infinite();
                if (bisector.IsDone)
                {
                    int nbsolution = bisector.SolutionCount;
                    for (int i = 1; i <= nbsolution; i++)
                    {
                        ConstraintBisec solution = bisector.ThisSolution(i);
                        Degenerate(ref solution, tolerance);
                        sense = false;
                        distanceptsol = Distance(apoint, solution, afirstvector, asecondvector, vecRef, adirection,
                                                 ref parameter, ref sense, out ok);

                        if (distanceptsol <= distancemini)
                        {
                            theSol = solution;
                            firstparameter = parameter;
                            thesense = sense;
                            distancemini = distanceptsol;
                        }
                    }
                    // OCCT: with no solution kept, the bisector stays unset (occt-issues.md).
                    if (theSol is not null)
                    {
                        ConstraintCurveType solType = theSol.ArcType();
                        Curve2D bisectorcurve;
                        if (solType == ConstraintCurveType.Lin)
                        {
                            // ----------------------------------------------------------------------------
                            // If the bisectrice is a line
                            //       => the point is on the circle.
                            //       If the part of bisectrice concerned is at the side of the center.
                            //       => the bisectrice is limited by the point and the center of the circle.
                            // Note : In this latter case the bisectrice is actually an ellipse of small null axis.
                            // ----------------------------------------------------------------------------

                            circlecenter = circle.Location;
                            line = theSol.Line();
                            secondparameter = global::Kerf.Engine.Geometry.ElCLib.Parameter(line, circlecenter);
                            bisectorcurve = new LineCurve2D(line);

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

                            _thebisector = new TrimmedCurve2D(bisectorcurve, firstparameter, secondparameter);
                        }
                        else if (solType == ConstraintCurveType.Cir)
                        {
                            bisectorcurve = new CircleCurve2D(theSol.Circle());
                            if (!thesense)
                            {
                                _thebisector = new TrimmedCurve2D(bisectorcurve, firstparameter - 2.0 * M_PI,
                                                                      firstparameter, thesense);
                            }
                            else
                            {
                                _thebisector = new TrimmedCurve2D(bisectorcurve, firstparameter,
                                                                      firstparameter + 2.0 * M_PI, thesense);
                            }
                        }
                        else if (solType == ConstraintCurveType.Hpr)
                        {
                            bisectorcurve = new HyperbolaCurve2D(theSol.Hyperbola());
                            if (!thesense)
                            {
                                _thebisector =
                                    new TrimmedCurve2D(bisectorcurve, firstparameter, -global::Kerf.Engine.Foundation.Precision.Infinite());
                            }
                            else
                            {
                                _thebisector =
                                    new TrimmedCurve2D(bisectorcurve, firstparameter, global::Kerf.Engine.Foundation.Precision.Infinite());
                            }
                        }
                        else if (solType == ConstraintCurveType.Ell)
                        {
                            bisectorcurve = new EllipseCurve2D(theSol.Ellipse());
                            if (!thesense)
                            {
                                _thebisector = new TrimmedCurve2D(bisectorcurve, firstparameter - 2.0 * M_PI,
                                                                      firstparameter, thesense);
                            }
                            else
                            {
                                _thebisector = new TrimmedCurve2D(bisectorcurve, firstparameter,
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
                LinPnt2dBisec bisector = new(line, asecondpoint.Pnt2d());

                ConstraintBisec solution = bisector.ThisSolution();
                Degenerate(ref solution, tolerance);
                ConstraintCurveType type = solution.ArcType();
                Curve2D bisectorcurve = null!;

                if (type == ConstraintCurveType.Lin)
                {
                    bisectorcurve = new LineCurve2D(solution.Line());
                }
                else if (type == ConstraintCurveType.Par)
                {
                    bisectorcurve = new ParabolaCurve2D(solution.Parabola());
                }
                sense = false;
                distanceptsol = Distance(apoint, solution, afirstvector, asecondvector, vecRef, adirection,
                                         ref parameter, ref sense, out ok);
                if (ok || !oncurve)
                {
                    firstparameter = parameter;
                    thesense = sense;
                }

                if (!thesense)
                {
                    _thebisector = new TrimmedCurve2D(bisectorcurve, firstparameter, -global::Kerf.Engine.Foundation.Precision.Infinite());
                }
                else
                {
                    _thebisector = new TrimmedCurve2D(bisectorcurve, firstparameter, global::Kerf.Engine.Foundation.Precision.Infinite());
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
    internal void Perform(Point2 afirstpoint, Curve2D asecondcurve, in Point2D apoint,
                          in Vector2D afirstvector, in Vector2D asecondvector, double adirection, double tolerance,
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
    /// <exception cref="NoSolutionException">The points coincide.</exception>
    internal void Perform(Point2 afirstpoint, Point2 asecondpoint, in Point2D apoint,
                          in Vector2D afirstvector, in Vector2D asecondvector, double adirection,
                          double tolerance = 0.0, bool oncurve = true)
    {
        CheckMutable();
        bool sense, ok;
        double parameter = 0.0;
        Vector2D vecRef = new(0.0, 0.0);

        Pnt2dBisec bisector = new(afirstpoint.Pnt2d(), asecondpoint.Pnt2d());
        Line2D line = bisector.ThisSolution();
        ConstraintBisec solution = new BLine(line);

        sense = false;
        Distance(apoint, solution, afirstvector, asecondvector, vecRef, adirection, ref parameter, ref sense, out ok);
        // OCCT: otherwise the bisector stays unset, and Bisector_Bisec dereferences it (occt-issues.md).
        if (ok || !oncurve)
        {
            Curve2D bisectorcurve = new LineCurve2D(line);
            if (!sense)
            {
                _thebisector = new TrimmedCurve2D(bisectorcurve, parameter, -global::Kerf.Engine.Foundation.Precision.Infinite());
            }
            else
            {
                _thebisector = new TrimmedCurve2D(bisectorcurve, parameter, global::Kerf.Engine.Foundation.Precision.Infinite());
            }
        }
    }

    /// <summary>False: an analytic bisector is never extended.</summary>
    public override bool IsExtendAtStart => false;

    /// <summary>False: an analytic bisector is never extended.</summary>
    public override bool IsExtendAtEnd => false;

    //=============================================================================
    //           The domain of the curve is the set of points that are closer to the
    //           than to its extremities.
    //           For the calculation the domain is extended. Extension of Epsilon1 of the
    //           First and the Last parameter of the curve.
    //=============================================================================
    /// <summary>
    /// Trims the bisector by the domain of the curve Cu: does nothing, OCCT's body is commented out.
    /// </summary>
    internal void SetTrim(Curve2D cu)
    {
        CheckMutable();
        // OCCT's body is commented out (the trimming by the limit lines of Cu's domain).
    }

    /// <summary>Trims the bisector to [uf, ul].</summary>
    internal void SetTrim(double uf, double ul)
    {
        CheckMutable();
        OwnBisector();
        _thebisector.SetTrim(uf, ul);
    }

    /// <summary>Reverses the bisector.</summary>
    protected internal override void Reverse()
    {
        CheckMutable();
        OwnBisector();
        _thebisector.Reverse();
    }

    /// <summary>The trimmed conic's reversed parameter.</summary>
    public override double ReversedParameter(double u) => _thebisector.ReversedParameter(u);

    /// <summary>The trimmed conic's IsCN.</summary>
    /// <exception cref="RangeErrorException">N &lt; 0.</exception>
    internal override bool IsCN(int n) => _thebisector.IsCN(n);

    /// <summary>A new, unfrozen bisector with a copy of the trimmed conic.</summary>
    public override Geometry2D Copy()
    {
        BisecAna c = new();
        c.Init((TrimmedCurve2D)_thebisector.Copy());
        return c;
    }

    /// <summary>Transforms the trimmed conic.</summary>
    protected internal override void Transform(in Transform2D t)
    {
        CheckMutable();
        OwnBisector();
        _thebisector.Transform(t);
    }

    /// <summary>The first parameter of the trimmed conic.</summary>
    public override double FirstParameter
    {
        get
        {
            //  modified by NIZHNY-EAP Thu Feb  3 17:23:42 2000 ___BEGIN___
            //  return thebisector->BasisCurve()->FirstParameter();
            return _thebisector.FirstParameter;
            //  modified by NIZHNY-EAP Thu Feb  3 17:23:48 2000 ___END___
        }
    }

    /// <summary>The last parameter of the trimmed conic.</summary>
    public override double LastParameter => _thebisector.LastParameter;

    /// <summary>True if the conic is closed.</summary>
    public override bool IsClosed => _thebisector.ChangeBasisCurve().IsClosed;

    /// <summary>True if the conic is periodic.</summary>
    public override bool IsPeriodic => _thebisector.ChangeBasisCurve().IsPeriodic;

    /// <summary>The trimmed conic's continuity.</summary>
    public override Continuity Continuity => _thebisector.Continuity;

    /// <summary>The conic's point at U (not limited to the trimmed range).</summary>
    internal override Point2D EvalD0(double u) => _thebisector.ChangeBasisCurve().EvalD0(u);

    /// <summary>The conic's point and first derivative at U.</summary>
    internal override ResD1 EvalD1(double u) => _thebisector.ChangeBasisCurve().EvalD1(u);

    /// <summary>The conic's point and first two derivatives at U.</summary>
    internal override ResD2 EvalD2(double u) => _thebisector.ChangeBasisCurve().EvalD2(u);

    /// <summary>The conic's point and first three derivatives at U.</summary>
    internal override ResD3 EvalD3(double u) => _thebisector.ChangeBasisCurve().EvalD3(u);

    /// <summary>The conic's derivative of order N at U.</summary>
    internal override Vector2D EvalDN(double u, int n) => _thebisector.ChangeBasisCurve().EvalDN(u, n);

    /// <summary>The conic (line, circle, ellipse, parabola or hyperbola), frozen.</summary>
    public Curve2D Geom2dCurve() => _thebisector.BasisCurve;

    /// <summary>The parameter of the point P on the conic.</summary>
    public override double Parameter(in Point2D p)
    {
        Hyperbola2D gphyperbola;
        Parabola2D gpparabola;
        Ellipse2D gpellipse;
        Circle2D gpcircle;
        Line2D gpline;

        Curve2D basisCurve = _thebisector.ChangeBasisCurve();
        Type type = basisCurve.GetType();

        if (type == typeof(LineCurve2D))
        {
            gpline = ((LineCurve2D)basisCurve).Line;
            return global::Kerf.Engine.Geometry.ElCLib.Parameter(gpline, p);
        }
        else if (type == typeof(CircleCurve2D))
        {
            gpcircle = ((CircleCurve2D)basisCurve).Circle;
            return global::Kerf.Engine.Geometry.ElCLib.Parameter(gpcircle, p);
        }
        else if (type == typeof(HyperbolaCurve2D))
        {
            gphyperbola = ((HyperbolaCurve2D)basisCurve).Hyperbola;
            return global::Kerf.Engine.Geometry.ElCLib.Parameter(gphyperbola, p);
        }
        else if (type == typeof(ParabolaCurve2D))
        {
            gpparabola = ((ParabolaCurve2D)basisCurve).Parabola;
            return global::Kerf.Engine.Geometry.ElCLib.Parameter(gpparabola, p);
        }
        else if (type == typeof(EllipseCurve2D))
        {
            gpellipse = ((EllipseCurve2D)basisCurve).Ellipse;
            return global::Kerf.Engine.Geometry.ElCLib.Parameter(gpellipse, p);
        }
        return 0.0;
    }

    /// <summary>The parameter of the start point: the first parameter of the trimmed conic.</summary>
    public double ParameterOfStartPoint() => _thebisector.FirstParameter;

    /// <summary>The parameter of the end point: the last parameter of the trimmed conic.</summary>
    public double ParameterOfEndPoint() => _thebisector.LastParameter;

    /// <summary>1: an analytic bisector is one interval.</summary>
    public override int IntervalCount => 1;

    /// <summary>The first parameter.</summary>
    /// <exception cref="OutOfRangeException">I is not 1.</exception>
    public override double IntervalFirst(int i)
    {
        if (i != 1)
        {
            throw new OutOfRangeException();
        }
        return FirstParameter;
    }

    /// <summary>The last parameter.</summary>
    /// <exception cref="OutOfRangeException">I is not 1.</exception>
    public override double IntervalLast(int i)
    {
        if (i != 1)
        {
            throw new OutOfRangeException();
        }
        return LastParameter;
    }

    /// <summary>Makes Bis the bisector (shared, as OCCT's handle; copied before this curve changes if frozen).</summary>
    internal void Init(TrimmedCurve2D bis)
    {
        CheckMutable();
        _thebisector = bis;
    }

    //=============================================================================
    //           if the bisectrice is an ellipse, a parabole or a degenerated ellipse.
    //=============================================================================
    private static bool Degenerate(ref ConstraintBisec aBisector, double tolerance)
    {
        bool degeneree = false;

        Hyperbola2D gphyperbola;
        Parabola2D gpparabola;
        Ellipse2D gpellipse;

        ConstraintBisec newBisector;

        ConstraintCurveType type = aBisector.ArcType();

        if (type == ConstraintCurveType.Hpr)
        {
            gphyperbola = aBisector.Hyperbola();

            // If the Hyperbola is degenerated, it is replaced by the straight line
            // with direction to the axis if symmetry.

            if (gphyperbola.MajorRadius < tolerance)
            {
                Line2D gpline = new(gphyperbola.YAxis);
                newBisector = new BLine(gpline);
                aBisector = newBisector;
                degeneree = true;
            }
            if (gphyperbola.MinorRadius < tolerance)
            {
                Line2D gpline = new(gphyperbola.XAxis);
                newBisector = new BLine(gpline);
                aBisector = newBisector;
                degeneree = true;
            }
        }
        else if (type == ConstraintCurveType.Par)
        {
            gpparabola = aBisector.Parabola();

            // If the parabole is degenerated, it is replaces by the straight
            // line starting at the Top and with direction of the axis of symmetry.

            if (gpparabola.Focal < tolerance)
            {
                Line2D gpline = new(gpparabola.MirrorAxis);
                newBisector = new BLine(gpline);
                aBisector = newBisector;
                degeneree = true;
            }
        }
        else if (type == ConstraintCurveType.Ell)
        {
            gpellipse = aBisector.Ellipse();

            // If the ellipse is degenerated, it is replaced by the straight line
            // defined by the great axis.

            if (gpellipse.MinorRadius < tolerance)
            {
                Line2D gpline = new(gpellipse.XAxis);
                newBisector = new BLine(gpline);
                aBisector = newBisector;
                degeneree = true;
            }
        }
        return degeneree;
    }

    // OCCT's Dump (std::cout) is left out.
}
