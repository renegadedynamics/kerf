// Port of: src/ModelingData/TKGeomBase/Extrema/Extrema_ExtElCS.hxx
// Port of: src/ModelingData/TKGeomBase/Extrema/Extrema_ExtElCS.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

//  Modified by skv - Thu Jul  7 14:37:05 2005 OCC9134

using static Occt.IntAna_ResultType;

namespace Occt;

/// <summary>
/// All the distances between an elementary curve and an elementary surface. These distances can be maximum or
/// minimum. OCCT implements line/plane, line/cylinder, line/sphere, circle/plane, circle/cylinder, circle/sphere and
/// hyperbola/plane; the other pairs throw <see cref="Standard_NotImplemented"/>.
/// </summary>
/// <remarks>
/// An algorithm object: construct it with the curve and the surface, or empty and call <c>Perform</c>. When the
/// result is parallel (<see cref="IsParallel"/>), only <see cref="SquareDistance"/> is meaningful: as in OCCT,
/// <see cref="Points"/> then reads arrays that this computation didn't fill (null, or those of an earlier
/// <c>Perform</c>).
/// </remarks>
public class Extrema_ExtElCS
{
    private bool myDone;
    private int myNbExt;
    private bool myIsPar;
    private NCollection_HArray1<double>? mySqDist;
    private NCollection_HArray1<Extrema_POnCurv>? myPoint1;
    private NCollection_HArray1<Extrema_POnSurf>? myPoint2;

    public Extrema_ExtElCS()
    {
        myDone = false;
        myIsPar = false;
        myNbExt = 0;
    }

    /// <summary>The distances between a line and a plane. The line can be on the plane or on a parallel plane.</summary>
    public Extrema_ExtElCS(in gp_Lin C, in gp_Pln S)
    {
        Perform(C, S);
    }

    public void Perform(in gp_Lin C, in gp_Pln S)
    {
        myDone = true;
        myIsPar = false;
        myNbExt = 0;

        if (C.Direction().IsNormal(S.Axis().Direction(), Precision.Angular()))
        {
            mySqDist = new NCollection_HArray1<double>(1, 1);
            mySqDist.SetValue(1, S.SquareDistance(C));
            myIsPar = true;
            myNbExt = 1;
        }
    }

    /// <summary>The distances between a line and a cylinder.</summary>
    public Extrema_ExtElCS(in gp_Lin C, in gp_Cylinder S)
    {
        Perform(C, S);
    }

    public void Perform(in gp_Lin C, in gp_Cylinder S)
    {
        myDone = false;
        myNbExt = 0;
        myIsPar = false;

        gp_Ax3 Pos = S.Position();

        bool isParallel = false;

        double radius = S.Radius();
        Extrema_ExtElC Extrem = new(new gp_Lin(Pos.Axis()), C, Precision.Angular());
        if (Extrem.IsParallel())
        {
            isParallel = true;
        }
        else
        {
            int i;

            Extrema_POnCurv myPOnC1, myPOnC2;
            Extrem.Points(1, out myPOnC1, out myPOnC2);
            gp_Pnt PonAxis = myPOnC1.Value();
            gp_Pnt PC = myPOnC2.Value();

            // line intersects the cylinder
            if (radius - PonAxis.Distance(PC) > Precision.PConfusion())
            {
                IntAna_Quadric theQuadric = new(S);
                IntAna_IntConicQuad Inters = new(C, theQuadric);
                if (Inters.IsDone() && Inters.IsInQuadric())
                {
                    isParallel = true;
                }
                else if (Inters.IsDone())
                {
                    myNbExt = Inters.NbPoints();
                    if (myNbExt > 0)
                    {
                        // Not more than 2 additional points from perpendiculars.
                        mySqDist = new NCollection_HArray1<double>(1, myNbExt + 2);
                        myPoint1 = new NCollection_HArray1<Extrema_POnCurv>(1, myNbExt + 2);
                        myPoint2 = new NCollection_HArray1<Extrema_POnSurf>(1, myNbExt + 2);
                        double u, v, w;
                        for (i = 1; i <= myNbExt; i++)
                        {
                            mySqDist.SetValue(i, 0.0);
                            gp_Pnt P_int = Inters.Point(i);
                            w = Inters.ParamOnConic(i);
                            Extrema_POnCurv PonC = new(w, P_int);
                            myPoint1.SetValue(i, PonC);
                            ElSLib.CylinderParameters(Pos, radius, P_int, out u, out v);
                            Extrema_POnSurf PonS = new(u, v, P_int);
                            myPoint2.SetValue(i, PonS);
                        }
                    }
                }
            }
            else
            {
                // line is tangent or outside of the cylinder
                Extrema_ExtPElS ExPS = new(PC, S, Precision.Confusion());
                if (ExPS.IsDone())
                {
                    myNbExt = ExPS.NbExt();
                    mySqDist = new NCollection_HArray1<double>(1, myNbExt);
                    myPoint1 = new NCollection_HArray1<Extrema_POnCurv>(1, myNbExt);
                    myPoint2 = new NCollection_HArray1<Extrema_POnSurf>(1, myNbExt);

                    for (i = 1; i <= myNbExt; i++)
                    {
                        myPoint1.SetValue(i, myPOnC2);
                        myPoint2.SetValue(i, ExPS.Point(i));
                        mySqDist.SetValue(i, (myPOnC2.Value()).SquareDistance(ExPS.Point(i).Value()));
                    }
                }
            }

            myDone = true;
        }

        if (isParallel)
        {
            // Line direction is similar to cylinder axis of rotation.
            // The case is possible when either extrema returned parallel status
            // or Intersection tool returned infinite number of solutions.
            // This is possible due to Intersection algorithm uses more precise
            // characteristics to consider given geometries parallel.
            // In the latter case there may be several extremas, thus we look for
            // the one with the lowest distance and use it as a final solution.
            mySqDist = new NCollection_HArray1<double>(1, 1);
            double aDist = Extrem.SquareDistance(1);
            int aNbExt = Extrem.NbExt();
            for (int i = 2; i <= aNbExt; i++)
            {
                double aD = Extrem.SquareDistance(i);
                if (aD < aDist)
                {
                    aDist = aD;
                }
            }

            aDist = Math.Sqrt(aDist) - radius;
            mySqDist.SetValue(1, aDist * aDist);
            myDone = true;
            myIsPar = true;
            myNbExt = 1;
        }
    }

    /// <summary>The distances between a line and a cone (not implemented in OCCT).</summary>
    public Extrema_ExtElCS(in gp_Lin C, in gp_Cone S)
    {
        Perform(C, S);
    }

    // void Extrema_ExtElCS::Perform(const gp_Lin& C,
    //			      const gp_Cone& S)
    /// <exception cref="Standard_NotImplemented">Always.</exception>
    public void Perform(in gp_Lin C, in gp_Cone S)
    {
        throw new Standard_NotImplemented();
    }

    /// <summary>The distances between a line and a sphere.</summary>
    public Extrema_ExtElCS(in gp_Lin C, in gp_Sphere S)
    {
        Perform(C, S);
    }

    public void Perform(in gp_Lin C, in gp_Sphere S)
    {
        // In case of intersection - return four points:
        // 2 intersection points and 2 perpendicular.
        // No intersection - only min and max.

        myDone = false;
        myNbExt = 0;
        myIsPar = false;
        int aStartIdx = 0;

        gp_Pnt aCenter = S.Location();

        Extrema_ExtPElC Extrem = new(aCenter, C, Precision.Angular(), RealFirst(), RealLast());

        int i;
        if (Extrem.IsDone() && Extrem.NbExt() > 0)
        {
            Extrema_POnCurv myPOnC1 = Extrem.Point(1);
            if (myPOnC1.Value().Distance(aCenter) <= S.Radius())
            {
                IntAna_IntConicQuad aLinSphere = new(C, new IntAna_Quadric(S));
                if (aLinSphere.IsDone())
                {
                    myNbExt = aLinSphere.NbPoints();
                    aStartIdx = myNbExt;
                    // Not more than 2 additional points from perpendiculars.
                    mySqDist = new NCollection_HArray1<double>(1, myNbExt + 2);
                    myPoint1 = new NCollection_HArray1<Extrema_POnCurv>(1, myNbExt + 2);
                    myPoint2 = new NCollection_HArray1<Extrema_POnSurf>(1, myNbExt + 2);

                    for (i = 1; i <= myNbExt; i++)
                    {
                        Extrema_POnCurv aCPnt = new(aLinSphere.ParamOnConic(i), aLinSphere.Point(i));

                        double u, v;
                        ElSLib.Parameters(S, aLinSphere.Point(i), out u, out v);
                        Extrema_POnSurf aSPnt = new(u, v, aLinSphere.Point(i));

                        myPoint1.SetValue(i, aCPnt);
                        myPoint2.SetValue(i, aSPnt);
                        mySqDist.SetValue(i, (aCPnt.Value()).SquareDistance(aSPnt.Value()));
                    }
                }
            }

            Extrema_ExtPElS ExPS = new(myPOnC1.Value(), S, Precision.Confusion());
            if (ExPS.IsDone())
            {
                if (aStartIdx == 0)
                {
                    myNbExt = ExPS.NbExt();
                    mySqDist = new NCollection_HArray1<double>(1, myNbExt);
                    myPoint1 = new NCollection_HArray1<Extrema_POnCurv>(1, myNbExt);
                    myPoint2 = new NCollection_HArray1<Extrema_POnSurf>(1, myNbExt);
                }
                else
                {
                    myNbExt += ExPS.NbExt();
                }

                for (i = aStartIdx + 1; i <= myNbExt; i++)
                {
                    myPoint1!.SetValue(i, myPOnC1);
                    myPoint2!.SetValue(i, ExPS.Point(i - aStartIdx));
                    mySqDist!.SetValue(i, (myPOnC1.Value()).SquareDistance(ExPS.Point(i - aStartIdx).Value()));
                }
            }
        }
        myDone = true;
    }

    /// <summary>The distances between a line and a torus (not implemented in OCCT).</summary>
    public Extrema_ExtElCS(in gp_Lin C, in gp_Torus S)
    {
        Perform(C, S);
    }

    // void Extrema_ExtElCS::Perform(const gp_Lin& C,
    //			      const gp_Torus& S)
    /// <exception cref="Standard_NotImplemented">Always.</exception>
    public void Perform(in gp_Lin C, in gp_Torus S)
    {
        throw new Standard_NotImplemented();
    }

    //        Circle-?

    /// <summary>The distances between a circle and a plane.</summary>
    public Extrema_ExtElCS(in gp_Circ C, in gp_Pln S)
    {
        Perform(C, S);
    }

    public void Perform(in gp_Circ C, in gp_Pln S)
    {
        myDone = true;
        myIsPar = false;
        myNbExt = 0;

        gp_Ax2 Pos = C.Position();
        gp_Dir NCirc = Pos.Direction();
        gp_Dir NPln = S.Axis().Direction();

        bool isParallel = false;

        if (NCirc.IsParallel(NPln, Precision.Angular()))
        {
            isParallel = true;
        }
        else
        {
            gp_Dir ExtLine = NCirc ^ NPln;
            ExtLine = ExtLine ^ NCirc;
            //
            gp_Dir XDir = Pos.XDirection();
            Span<double> T = stackalloc double[2];
            T[0] = XDir.AngleWithRef(ExtLine, NCirc);
            if (T[0] < 0.0)
            {
                // Put in period
                T[0] += M_PI;
            }
            T[1] = T[0] + M_PI;
            //
            myNbExt = 2;
            // Check intersection
            IntAna_IntConicQuad anInter = new(C, S, Precision.Angular(), Precision.Confusion());

            if (anInter.IsDone() && anInter.IsInQuadric())
            {
                isParallel = true;
            }
            else if (anInter.IsDone())
            {
                if (anInter.NbPoints() > 1)
                {
                    myNbExt += anInter.NbPoints();
                }
            }

            if (!isParallel)
            {
                myPoint1 = new NCollection_HArray1<Extrema_POnCurv>(1, myNbExt);
                mySqDist = new NCollection_HArray1<double>(1, myNbExt);
                myPoint2 = new NCollection_HArray1<Extrema_POnSurf>(1, myNbExt);

                int i;
                gp_Pnt PC, PP;
                double U, V;
                Extrema_POnCurv POnC = new();
                Extrema_POnSurf POnS = new();
                for (i = 0; i < 2; ++i)
                {
                    PC = ElCLib.CircleValue(T[i], C.Position(), C.Radius());
                    POnC.SetValues(T[i], PC);
                    myPoint1.SetValue(i + 1, POnC);
                    ElSLib.PlaneParameters(S.Position(), PC, out U, out V);
                    PP = ElSLib.PlaneValue(U, V, S.Position());
                    POnS.SetParameters(U, V, PP);
                    myPoint2.SetValue(i + 1, POnS);
                    mySqDist.SetValue(i + 1, PC.SquareDistance(PP));
                }
                //
                if (myNbExt > 2)
                {
                    // Add intersection points
                    for (i = 1; i <= anInter.NbPoints(); ++i)
                    {
                        double t = anInter.ParamOnConic(i);
                        PC = ElCLib.CircleValue(t, C.Position(), C.Radius());
                        POnC.SetValues(t, PC);
                        myPoint1.SetValue(i + 2, POnC);
                        ElSLib.PlaneParameters(S.Position(), PC, out U, out V);
                        PP = ElSLib.PlaneValue(U, V, S.Position());
                        POnS.SetParameters(U, V, PP);
                        myPoint2.SetValue(i + 2, POnS);
                        mySqDist.SetValue(i + 2, PC.SquareDistance(PP));
                    }
                }
            }
        }

        if (isParallel)
        {
            mySqDist = new NCollection_HArray1<double>(1, 1);
            mySqDist.SetValue(1, S.SquareDistance(C.Location()));
            myIsPar = true;
            myNbExt = 1;
        }
    }

    /// <summary>The distances between a circle and a cylinder.</summary>
    public Extrema_ExtElCS(in gp_Circ C, in gp_Cylinder S)
    {
        Perform(C, S);
    }

    //  Modified by skv - Thu Jul  7 14:37:05 2005 OCC9134 Begin
    // Implementation of the method.
    public void Perform(in gp_Circ C, in gp_Cylinder S)
    {
        myDone = false;
        myIsPar = false;
        myNbExt = 0;

        // Get an axis line of the cylinder.
        gp_Lin anAxis = new(S.Axis());

        // Compute extrema between the circle and the line.
        Extrema_ExtElC anExtC = new(anAxis, C, 0.0);

        if (!anExtC.IsDone())
        {
            return;
        }

        bool isParallel = false;

        if (anExtC.IsParallel())
        {
            isParallel = true;
        }
        else
        {
            int aNbExt = anExtC.NbExt();
            int i;
            int aCurI = 1;
            double aTolConf = Precision.Confusion();
            double aCylRad = S.Radius();

            // Check whether two objects have intersection points
            IntAna_Quadric aCylQuad = new(S);
            IntAna_IntConicQuad aCircCylInter = new(C, aCylQuad);
            int aNbInter = 0;
            if (aCircCylInter.IsDone() && aCircCylInter.IsInQuadric())
            {
                isParallel = true;
            }
            else if (aCircCylInter.IsDone())
            {
                aNbInter = aCircCylInter.NbPoints();
            }

            if (!isParallel)
            {
                // Compute the extremas.
                myNbExt = 2 * aNbExt + aNbInter;
                mySqDist = new NCollection_HArray1<double>(1, myNbExt);
                myPoint1 = new NCollection_HArray1<Extrema_POnCurv>(1, myNbExt);
                myPoint2 = new NCollection_HArray1<Extrema_POnSurf>(1, myNbExt);

                for (i = 1; i <= aNbExt; i++)
                {
                    Extrema_POnCurv aPOnAxis;
                    Extrema_POnCurv aPOnCirc;
                    double aSqDist = anExtC.SquareDistance(i);
                    double aDist = Math.Sqrt(aSqDist);

                    anExtC.Points(i, out aPOnAxis, out aPOnCirc);

                    if (aSqDist <= (aTolConf * aTolConf))
                    {
                        myNbExt -= 2;
                        continue;
                    }

                    gp_Dir aDir = new(aPOnAxis.Value().XYZ().Subtracted(aPOnCirc.Value().XYZ()));
                    ReadOnlySpan<double> aShift = [aDist + aCylRad, aDist - aCylRad];
                    int j;

                    for (j = 0; j < 2; j++)
                    {
                        gp_Vec aVec = new(aDir);
                        gp_Pnt aPntOnCyl;

                        aVec.Multiply(aShift[j]);
                        aPntOnCyl = aPOnCirc.Value().Translated(aVec);

                        double aU;
                        double aV;

                        ElSLib.Parameters(S, aPntOnCyl, out aU, out aV);

                        Extrema_POnSurf aPOnSurf = new(aU, aV, aPntOnCyl);

                        myPoint1.SetValue(aCurI, aPOnCirc);
                        myPoint2.SetValue(aCurI, aPOnSurf);
                        mySqDist.SetValue(aCurI++, aShift[j] * aShift[j]);
                    }
                }

                // Adding intersection points to the list of extremas
                for (i = 1; i <= aNbInter; i++)
                {
                    double aU;
                    double aV;

                    gp_Pnt aInterPnt = aCircCylInter.Point(i);

                    aU = ElCLib.Parameter(C, aInterPnt);
                    Extrema_POnCurv aPOnCirc = new(aU, aInterPnt);

                    ElSLib.Parameters(S, aInterPnt, out aU, out aV);
                    Extrema_POnSurf aPOnCyl = new(aU, aV, aInterPnt);
                    myPoint1.SetValue(aCurI, aPOnCirc);
                    myPoint2.SetValue(aCurI, aPOnCyl);
                    mySqDist.SetValue(aCurI++, 0.0);
                }
            }
        }

        myDone = true;

        if (isParallel)
        {
            // The case is possible when either extrema returned parallel status
            // or Intersection tool returned infinite number of solutions.
            // This is possible due to Intersection algorithm uses more precise
            // characteristics to consider given geometries parallel.
            // In the latter case there may be several extremas, thus we look for
            // the one with the lowest distance and use it as a final solution.

            myIsPar = true;
            myNbExt = 1;
            mySqDist = new NCollection_HArray1<double>(1, 1);
            double aDist = anExtC.SquareDistance(1);

            int aNbExt = anExtC.NbExt();
            for (int i = 2; i <= aNbExt; i++)
            {
                double aD = anExtC.SquareDistance(i);
                if (aD < aDist)
                {
                    aDist = aD;
                }
            }

            aDist = Math.Sqrt(aDist) - S.Radius();
            mySqDist.SetValue(1, aDist * aDist);
        }
    }

    //  Modified by skv - Thu Jul  7 14:37:05 2005 OCC9134 End

    /// <summary>The distances between a circle and a cone (not implemented in OCCT).</summary>
    public Extrema_ExtElCS(in gp_Circ C, in gp_Cone S)
    {
        Perform(C, S);
    }

    // void Extrema_ExtElCS::Perform(const gp_Circ& C,
    //			 const gp_Cone& S)
    /// <exception cref="Standard_NotImplemented">Always.</exception>
    public void Perform(in gp_Circ C, in gp_Cone S)
    {
        throw new Standard_NotImplemented();
    }

    /// <summary>The distances between a circle and a sphere: the minimal ones only.</summary>
    public Extrema_ExtElCS(in gp_Circ C, in gp_Sphere S)
    {
        Perform(C, S);
    }

    public void Perform(in gp_Circ C, in gp_Sphere S)
    {
        myDone = false;
        myIsPar = false;
        myNbExt = 0;

        if (new gp_Lin(C.Axis()).SquareDistance(S.Location()) < Precision.SquareConfusion())
        {
            // Circle and sphere are parallel
            myIsPar = true;
            myDone = true;
            myNbExt = 1;

            // Compute distance from circle to the sphere
            double aSqDistLoc = C.Location().SquareDistance(S.Location());
            double aSqDist = aSqDistLoc + C.Radius() * C.Radius();
            double aDist = Math.Sqrt(aSqDist) - S.Radius();
            mySqDist = new NCollection_HArray1<double>(1, 1);
            mySqDist.SetValue(1, aDist * aDist);
            return;
        }

        // Intersect sphere with circle's plane
        gp_Pln CPln = new(C.Location(), C.Axis().Direction());
        IntAna_QuadQuadGeo anInter = new(CPln, S);
        if (!anInter.IsDone())
        {
            // not done
            return;
        }

        if (anInter.TypeInter() != IntAna_Circle)
        {
            // Intersection is empty or just a point.
            // The parallel case has already been considered,
            // thus, here we have to find only one minimal solution
            myNbExt = 1;
            myDone = true;

            mySqDist = new NCollection_HArray1<double>(1, 1);
            myPoint1 = new NCollection_HArray1<Extrema_POnCurv>(1, 1);
            myPoint2 = new NCollection_HArray1<Extrema_POnSurf>(1, 1);

            // Compute parameter on circle
            double aT = ElCLib.Parameter(C, S.Location());
            // Compute point on circle
            gp_Pnt aPOnC = ElCLib.Value(aT, C);

            // Compute parameters on sphere
            double aU, aV;
            ElSLib.Parameters(S, aPOnC, out aU, out aV);
            // Compute point on sphere
            gp_Pnt aPOnS = ElSLib.Value(aU, aV, S);

            // Save solution
            myPoint1.SetValue(1, new Extrema_POnCurv(aT, aPOnC));
            myPoint2.SetValue(1, new Extrema_POnSurf(aU, aV, aPOnS));
            mySqDist.SetValue(1, aPOnC.SquareDistance(aPOnS));
            return;
        }

        // Here, the intersection is a circle

        // Intersection circle
        gp_Circ aCInt = anInter.Circle(1);

        // Perform intersection of the input circle with the intersection circle
        Extrema_ExtElC anExtC = new(C, aCInt);
        bool isExtremaCircCircValid = anExtC.IsDone() // Check if intersection is done
                                      && !anExtC.IsParallel() // Parallel case has already been considered
                                      && anExtC.NbExt() > 0; // Check that some solutions have been found
        if (!isExtremaCircCircValid)
        {
            // not done
            return;
        }

        myDone = true;

        // Few solutions
        double aNbExt = anExtC.NbExt();
        // Find the minimal distance
        double aMinSqDist = RealLast();
        for (int i = 1; i <= aNbExt; ++i)
        {
            double aSqDist = anExtC.SquareDistance(i);
            if (aSqDist < aMinSqDist)
            {
                aMinSqDist = aSqDist;
            }
        }

        // Collect all solutions close to the minimal one
        NCollection_List<int> aSols = new();
        for (int i = 1; i <= aNbExt; ++i)
        {
            double aDiff = anExtC.SquareDistance(i) - aMinSqDist;
            if (aDiff < Precision.SquareConfusion())
            {
                aSols.Append(i);
            }
        }

        // Save all minimal solutions
        myNbExt = aSols.Extent();

        mySqDist = new NCollection_HArray1<double>(1, myNbExt);
        myPoint1 = new NCollection_HArray1<Extrema_POnCurv>(1, myNbExt);
        myPoint2 = new NCollection_HArray1<Extrema_POnSurf>(1, myNbExt);

        NCollection_List<int>.Iterator it = new(aSols);
        for (int iSol = 1; it.More(); it.Next(), ++iSol)
        {
            Extrema_POnCurv P1, P2;
            anExtC.Points(it.Value(), out P1, out P2);

            // Compute parameters on sphere
            double aU, aV;
            ElSLib.Parameters(S, P1.Value(), out aU, out aV);
            // Compute point on sphere
            gp_Pnt aPOnS = ElSLib.Value(aU, aV, S);

            // Save solution
            myPoint1.SetValue(iSol, P1);
            myPoint2.SetValue(iSol, new Extrema_POnSurf(aU, aV, aPOnS));
            mySqDist.SetValue(iSol, P1.Value().SquareDistance(aPOnS));
        }
    }

    /// <summary>The distances between a circle and a torus (not implemented in OCCT).</summary>
    public Extrema_ExtElCS(in gp_Circ C, in gp_Torus S)
    {
        Perform(C, S);
    }

    // void Extrema_ExtElCS::Perform(const gp_Circ& C,
    //			      const gp_Torus& S)
    /// <exception cref="Standard_NotImplemented">Always.</exception>
    public void Perform(in gp_Circ C, in gp_Torus S)
    {
        throw new Standard_NotImplemented();
    }

    /// <summary>The distances between a hyperbola and a plane.</summary>
    public Extrema_ExtElCS(in gp_Hypr C, in gp_Pln S)
    {
        Perform(C, S);
    }

    public void Perform(in gp_Hypr C, in gp_Pln S)
    {
        myDone = true;
        myIsPar = false;
        myNbExt = 0;

        gp_Ax2 Pos = C.Position();
        gp_Dir NHypr = Pos.Direction();
        gp_Dir NPln = S.Axis().Direction();

        if (NHypr.IsParallel(NPln, Precision.Angular()))
        {
            mySqDist = new NCollection_HArray1<double>(1, 1);
            mySqDist.SetValue(1, S.SquareDistance(C.Location()));
            myIsPar = true;
            myNbExt = 1;
        }
        else
        {
            gp_Dir XDir = Pos.XDirection();
            gp_Dir YDir = Pos.YDirection();

            double A = C.MinorRadius() * (NPln.Dot(YDir));
            double B = C.MajorRadius() * (NPln.Dot(XDir));

            if (Math.Abs(B) > Math.Abs(A))
            {
                double T = -0.5 * Math.Log((A + B) / (B - A));
                gp_Pnt Ph = ElCLib.HyperbolaValue(T, Pos, C.MajorRadius(), C.MinorRadius());
                Extrema_POnCurv PC = new(T, Ph);
                myPoint1 = new NCollection_HArray1<Extrema_POnCurv>(1, 1);
                myPoint1.SetValue(1, PC);

                mySqDist = new NCollection_HArray1<double>(1, 1);
                mySqDist.SetValue(1, S.SquareDistance(Ph));

                double U, V;
                ElSLib.PlaneParameters(S.Position(), Ph, out U, out V);
                gp_Pnt Pp = ElSLib.PlaneValue(U, V, S.Position());
                Extrema_POnSurf PS = new(U, V, Pp);
                myPoint2 = new NCollection_HArray1<Extrema_POnSurf>(1, 1);
                myPoint2.SetValue(1, PS);

                myNbExt = 1;
            }
        }
    }

    /// <summary>True if the distances are found.</summary>
    public bool IsDone() => myDone;

    /// <summary>The number of extremum distances.</summary>
    /// <exception cref="StdFail_NotDone">The computation was not done.</exception>
    public int NbExt()
    {
        if (!IsDone())
        {
            throw new StdFail_NotDone();
        }
        return myNbExt;
    }

    /// <summary>The value of the Nth extremum square distance.</summary>
    /// <exception cref="Standard_OutOfRange">N is not in [1, NbExt].</exception>
    public double SquareDistance(int N = 1)
    {
        if (N < 1 || N > NbExt())
        {
            throw new Standard_OutOfRange();
        }

        return mySqDist!.Value(N);
    }

    /// <summary>The points of the Nth extremum distance: P1 on the curve, P2 on the surface.</summary>
    /// <exception cref="Standard_OutOfRange">N is not in [1, NbExt].</exception>
    public void Points(int N, out Extrema_POnCurv P1, out Extrema_POnSurf P2)
    {
        if (N < 1 || N > NbExt())
        {
            throw new Standard_OutOfRange();
        }

        // OCCT: a parallel result fills no points, so this reads null handles or those of an earlier Perform
        P1 = myPoint1!.Value(N);
        P2 = myPoint2!.Value(N);
    }

    /// <summary>True if the curve is on a parallel surface.</summary>
    /// <exception cref="StdFail_NotDone">The computation was not done.</exception>
    public bool IsParallel()
    {
        if (!IsDone())
        {
            throw new StdFail_NotDone();
        }
        return myIsPar;
    }
}
