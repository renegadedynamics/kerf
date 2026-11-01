// Port of: src/ModelingAlgorithms/TKShHealing/ShapeCustom/ShapeCustom_Surface.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeCustom/ShapeCustom_Surface.lxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeCustom/ShapeCustom_Surface.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GeomAbs_SurfaceType;

namespace Occt;

/// <summary>
/// Converts a B-spline or Bezier surface to an analytical surface (plane, cylinder, cone, sphere, torus) where one
/// fits within a tolerance, or a closed B-spline surface to its periodic form.
/// </summary>
/// <remarks>A working object (decision 21).</remarks>
public sealed class ShapeCustom_Surface
{
    private Geom_Surface? mySurf;
    private double myGap;

    /// <summary>An empty tool.</summary>
    public ShapeCustom_Surface()
    {
        myGap = 0;
    }

    /// <summary>A tool on the surface.</summary>
    public ShapeCustom_Surface(Geom_Surface? S)
    {
        myGap = 0;
        Init(S);
    }

    /// <summary>Sets the surface.</summary>
    public void Init(Geom_Surface? S)
    {
        mySurf = S;
    }

    /// <summary>The largest deviation of the converted surface from the original found by the last <see cref="ConvertToAnalytical"/>.</summary>
    public double Gap()
    {
        return myGap;
    }

    /// <summary>
    /// Tries to convert the surface (a B-spline or Bezier) to an analytic one within <paramref name="tol"/>, and
    /// returns it, or null. With <paramref name="substitute"/>, the result replaces the tool's surface. The
    /// parametrisation isn't kept, so pcurves must be redone.
    /// </summary>
    public Geom_Surface? ConvertToAnalytical(double tol, bool substitute)
    {
        Geom_Surface? newSurf = null;

        int nUP, nVP, nCP, i, j, UDeg, VDeg;
        double U1, U2, V1, V2, C1, C2, DU, DV, U = 0, V = 0;
        Geom_Curve? iso = null;
        bool uClosed = true;

        // seuls cas traites : BSpline et Bezier
        Geom_BSplineSurface? theBSplneS = mySurf as Geom_BSplineSurface;
        if (theBSplneS is null)
        {
            Geom_BezierSurface? theBezierS = mySurf as Geom_BezierSurface;
            if (theBezierS is not null)
            { // Bezier :
                nUP = theBezierS.NbUPoles();
                nVP = theBezierS.NbVPoles();
                UDeg = theBezierS.UDegree();
                VDeg = theBezierS.VDegree();
            }
            else
            {
                return newSurf; // non reconnu : terminus
            }
        }
        else
        { // BSpline :
            nUP = theBSplneS.NbUPoles();
            nVP = theBSplneS.NbVPoles();
            UDeg = theBSplneS.UDegree();
            VDeg = theBSplneS.VDegree();
        }

        mySurf!.Bounds(out U1, out U2, out V1, out V2);
        NCollection_Array1<gp_Pnt> p1 = new(1, 3), p2 = new(1, 3), p3 = new(1, 3);
        NCollection_Array1<double> R = new(1, 3);
        gp_Pnt origPnt, resPnt;
        gp_Vec origD1U = new gp_Vec(), resD1U, resD1V;

        bool aCySpCo = false;
        bool aToroid = false;
        bool aPlanar = false;

        if (nUP == 2 && nVP == 2)
        {
            if (UDeg == 1 && VDeg == 1)
            {
                aPlanar = true;
            }
        }
        else if (mySurf.IsUClosed())
        { // VRAI IsUClosed
            if (mySurf.IsVClosed())
            {
                aToroid = true;
            }
            else
            {
                aCySpCo = true;
            }
        }
        else
        {
            if (mySurf.IsVClosed())
            { // VRAI IsVClosed
                aCySpCo = true;
                uClosed = false;
            }
        }

        if (aPlanar)
        {
            //    NearestPlane ...
            NCollection_Array1<gp_Pnt> Pnts = new(1, 4);
            Pnts.SetValue(1, mySurf.Value(U1, V1));
            Pnts.SetValue(2, mySurf.Value(U2, V1));
            Pnts.SetValue(3, mySurf.Value(U1, V2));
            Pnts.SetValue(4, mySurf.Value(U2, V2));
            gp_Pln aPln = new gp_Pln(); // double Dmax;
            bool It = ShapeAnalysis_Geom.NearestPlane(Pnts, ref aPln, out myGap /*Dmax*/);

            //  ICI, on fabrique le plan, et zou
            if (!It || myGap /*Dmax*/ > tol)
            {
                return newSurf; //  pas un plan
            }

            //    IL RESTE a verifier l orientation ...
            //    On regarde sur chaque surface les vecteurs P(U0->U1),P(V0->V1)
            //    On prend la normale : les deux normales doivent etre dans le meme sens
            //    Sinon, inverser la normale (pas le Pln entier !) et refaire la Plane
            newSurf = new Geom_Plane(aPln);
            gp_Vec uold = new gp_Vec(Pnts[1], Pnts[2]);
            gp_Vec vold = new gp_Vec(Pnts[1], Pnts[3]);
            gp_Vec nold = uold.Crossed(vold);
            gp_Vec unew = new gp_Vec(newSurf.Value(U1, V1), newSurf.Value(U2, V1));
            gp_Vec vnew = new gp_Vec(newSurf.Value(U1, V1), newSurf.Value(U1, V2));
            gp_Vec nnew = unew.Crossed(vnew);
            if (nold.Dot(nnew) < 0.0)
            {
                gp_Ax3 ax3 = aPln.Position();
                ax3.ZReverse();
                ax3.XReverse();
                aPln = new gp_Pln(ax3);
                newSurf = new Geom_Plane(aPln);
            }

            if (substitute)
            {
                Init(newSurf);
            }
            return newSurf;
        }
        else if (aCySpCo)
        {
            if (!uClosed)
            {
                C1 = U1;
                C2 = U2;
                U1 = V1;
                U2 = V2;
                V1 = C1;
                V2 = C2;
                nCP = nUP;
                nUP = nVP;
                nVP = nCP;
            }

            for (i = 1; i <= 3; i++)
            {
                if (i == 1)
                {
                    V = V1;
                }
                else if (i == 2)
                {
                    V = V2;
                }
                else if (i == 3)
                {
                    V = 0.5 * (V1 + V2);
                }

                if (uClosed)
                {
                    iso = mySurf.VIso(V);
                }
                else
                {
                    iso = mySurf.UIso(V);
                }

                iso.D0(U1, out p1[i]);
                iso.D0(0.5 * (U1 + U2), out p2[i]);
                p3[i].SetCoord(0.5 * (p1[i].X() + p2[i].X()),
                               0.5 * (p1[i].Y() + p2[i].Y()),
                               0.5 * (p1[i].Z() + p2[i].Z()));
                R[i] = p3[i].Distance(p1[i]);
            }

            // OCCT: the last iso built, at parameter 0 rather than U1 (occt-issues.md)
            iso!.D1(0.0, out origPnt, out origD1U);
            gp_Vec xVec = new gp_Vec(p3[3], p1[3]);
            gp_Vec aVec = new gp_Vec(p3[1], p3[2]);
            //      gp_Dir xDir(xVec);  ne sert pas. Null si R3 = 0
            gp_Dir aDir = new gp_Dir(aVec);
            gp_Ax3 aAx3 = new gp_Ax3(p3[1], aDir, new gp_Dir(xVec));
            //  CKY  3-FEV-1997 : verification du sens de description
            // C++ converts origD1U to a gp_Dir implicitly
            if (aAx3.YDirection().Dot(new gp_Dir(origD1U)) < 0)
            {
                aAx3.YReverse(); //  mais X reste !
            }

            if (nVP > 2)
            {
                if ((Math.Abs(R[1]) < tol) && (Math.Abs(R[2]) < tol) && (Math.Abs(R[3]) > tol))
                {
                    Geom_SphericalSurface anObject = new Geom_SphericalSurface(aAx3, R[3]);
                    if (!uClosed)
                    {
                        anObject.UReverse();
                    }
                    newSurf = anObject;
                }
            }
            else if (nVP == 2)
            {
                if (Math.Abs(R[2] - R[1]) < tol)
                {
                    Geom_CylindricalSurface anObject = new Geom_CylindricalSurface(aAx3, R[1]);
                    if (!uClosed)
                    {
                        anObject.UReverse();
                    }
                    newSurf = anObject;
                }
                else
                {
                    gp_Vec aVec2 = new gp_Vec(p1[1], p1[2]);
                    double angle = aVec.Angle(aVec2);
                    if (R[1] < R[2])
                    {
                        Geom_ConicalSurface anObject = new Geom_ConicalSurface(aAx3, angle, R[1]);
                        // if (!uClosed) anObject->UReverse();
                        anObject.UReverse();
                        newSurf = anObject;
                    }
                    else
                    {
                        aDir.Reverse();
                        gp_Vec anotherXVec = new gp_Vec(p3[2], p1[2]);
                        gp_Dir anotherXDir = new gp_Dir(anotherXVec);
                        gp_Ax3 anotherAx3 = new gp_Ax3(p3[2], aDir, anotherXDir);
                        Geom_ConicalSurface anObject = new Geom_ConicalSurface(anotherAx3, angle, R[2]);
                        // if (!uClosed) anObject->UReverse();
                        anObject.UReverse();
                        newSurf = anObject;
                    }
                }
            }
        }
        else if (aToroid)
        {
            // test by iso U and isoV
            bool isFound = false;
            for (j = 1; (j <= 2) && !isFound; j++)
            {
                if (j == 2)
                {
                    C1 = U1;
                    C2 = U2;
                    U1 = V1;
                    U2 = V2;
                    V1 = C1;
                    V2 = C2;
                }
                for (i = 1; i <= 3; i++)
                {
                    if (i == 1)
                    {
                        U = U1;
                    }
                    else if (i == 2)
                    {
                        U = 0.5 * (U1 + U2);
                    }
                    else if (i == 3)
                    {
                        U = 0.25 * (U1 + U2);
                    }

                    iso = mySurf.UIso(U);

                    iso.D0(V1, out p1[i]);
                    iso.D0(0.5 * (V1 + V2), out p2[i]);
                    p3[i].SetCoord(0.5 * (p1[i].X() + p2[i].X()),
                                   0.5 * (p1[i].Y() + p2[i].Y()),
                                   0.5 * (p1[i].Z() + p2[i].Z()));
                    R[i] = p3[i].Distance(p1[i]);
                }
                if ((Math.Abs(R[1] - R[2]) < tol) && (Math.Abs(R[1] - R[3]) < tol))
                {
                    gp_Pnt p10 = new gp_Pnt(0.5 * (p3[1].X() + p3[2].X()),
                                            0.5 * (p3[1].Y() + p3[2].Y()),
                                            0.5 * (p3[1].Z() + p3[2].Z()));
                    gp_Vec aVec = new gp_Vec(p10, p3[1]);
                    gp_Vec aVec2 = new gp_Vec(p10, p3[3]);
                    double RR1 = R[1], RR2 = R[2], RR3;
                    aVec.Cross(aVec2);

                    if (aVec.Magnitude() <= gp.Resolution())
                    {
                        aVec.SetCoord(0.0, 0.0, 1.0);
                    }

                    gp_Dir aDir = new gp_Dir(aVec);

                    gp_Ax3 aAx3 = new gp_Ax3(p10, aDir);
                    RR1 = p10.Distance(p3[1]);
                    //          modif empirique (pourtant NON DEMONTREE) : inverser roles RR1,RR2
                    //          CKY, 24-JAN-1997
                    if (RR1 < RR2)
                    {
                        RR3 = RR1;
                        RR1 = RR2;
                        RR2 = RR3;
                    }
                    Geom_ToroidalSurface anObject = new Geom_ToroidalSurface(aAx3, RR1, RR2);
                    if (j == 2)
                    {
                        anObject.UReverse();
                    }
                    anObject.D1(0.0, 0.0, out resPnt, out resD1U, out resD1V);
                    newSurf = anObject;
                    isFound = true;
                }
            }
        }
        if (newSurf is null)
        {
            return newSurf;
        }

        //---------------------------------------------------------------------
        //                 verification
        //---------------------------------------------------------------------

        GeomAdaptor_Surface NHS = new GeomAdaptor_Surface(newSurf);
        GeomAdaptor_Surface SurfAdapt = NHS;

        const int NP = 21;
        double S = 0.0, T = 0.0; // U,V deja fait
        gp_Pnt P3d, P3d2;
        bool onSurface = true;

        double dis;
        myGap = 0.0;

        DU = (U2 - U1) / (NP - 1);
        DV = (V2 - V1) / (NP - 1);
        for (j = 1; (j <= NP) && onSurface; j++)
        {
            V = V1 + DV * (j - 1);

            if (uClosed)
            {
                iso = mySurf.VIso(V);
            }
            else
            {
                iso = mySurf.UIso(V);
            }

            for (i = 1; i <= NP; i++)
            {
                U = U1 + DU * (i - 1);
                iso.D0(U, out P3d);
                switch (SurfAdapt.GetType())
                {
                    case GeomAbs_Cylinder:
                    {
                        gp_Cylinder Cylinder = SurfAdapt.Cylinder();
                        ElSLib.Parameters(Cylinder, P3d, out S, out T);
                        break;
                    }
                    case GeomAbs_Cone:
                    {
                        gp_Cone Cone = SurfAdapt.Cone();
                        ElSLib.Parameters(Cone, P3d, out S, out T);
                        break;
                    }
                    case GeomAbs_Sphere:
                    {
                        gp_Sphere Sphere = SurfAdapt.Sphere();
                        ElSLib.Parameters(Sphere, P3d, out S, out T);
                        break;
                    }
                    case GeomAbs_Torus:
                    {
                        gp_Torus Torus = SurfAdapt.Torus();
                        ElSLib.Parameters(Torus, P3d, out S, out T);
                        break;
                    }
                    default:
                        break;
                }

                newSurf!.D0(S, T, out P3d2);

                dis = P3d.Distance(P3d2);
                if (dis > myGap)
                {
                    myGap = dis;
                }

                if (dis > tol)
                {
                    onSurface = false;
                    newSurf = null;
                    // The presumption is rejected
                    break;
                }
            }
        }
        // OCCT: the adaptor is never null, so a rejected surface (null) replaces the tool's surface with substitute
        // (occt-issues.md)
        if (substitute && NHS is not null)
        {
            Init(newSurf);
        }
        return newSurf;
    }

    /// <summary>
    /// Tries to convert the surface (a B-spline closed in U or V within <paramref name="preci"/>) to its periodic
    /// form, and returns the result, or null if nothing was done. With <paramref name="substitute"/>, the result
    /// replaces the tool's surface.
    /// </summary>
    /// <remarks>
    /// OCCT makes the surface itself periodic when its knots needn't be rearranged; the port makes a copy of a frozen
    /// surface periodic instead (decision 974), so the caller's surface never changes.
    /// </remarks>
    public Geom_Surface? ConvertToPeriodic(bool substitute, double preci = -1)
    {
        Geom_Surface? newSurf = null;
        Geom_BSplineSurface? BSpl = mySurf as Geom_BSplineSurface;
        if (BSpl is null)
        {
            return newSurf;
        }

        ShapeAnalysis_Surface sas = new ShapeAnalysis_Surface(mySurf);
        bool uclosed = sas.IsUClosed(preci);
        bool vclosed = sas.IsVClosed(preci);

        if (!uclosed && !vclosed)
        {
            return newSurf;
        }

        bool converted = false; //: p6

        if (uclosed && !BSpl.IsUPeriodic() && BSpl.NbUPoles() > 3)
        {
            bool set = true;
            // if degree+1 at ends, first change it to 1 by rearranging knots
            if (BSpl.UMultiplicity(1) == BSpl.UDegree() + 1
                && BSpl.UMultiplicity(BSpl.NbUKnots()) == BSpl.UDegree() + 1)
            {
                NCollection_Array2<gp_Pnt> oldPoles = BSpl.InternalPoles();
                NCollection_Array2<double> oldWeights = BSpl.InternalWeightsArray();
                int nbUKnots = BSpl.NbUKnots();
                NCollection_Array1<double> oldUKnots = BSpl.InternalUKnots();
                NCollection_Array1<double> oldVKnots = BSpl.InternalVKnots();
                NCollection_Array1<int> oldUMults = BSpl.InternalUMultiplicities();
                NCollection_Array1<int> oldVMults = BSpl.InternalVMultiplicities();

                NCollection_Array1<double> newUKnots = new(1, nbUKnots + 2);
                NCollection_Array1<int> newUMults = new(1, nbUKnots + 2);
                double a =
                    0.5 * (BSpl.UKnot(2) - BSpl.UKnot(1) + BSpl.UKnot(nbUKnots) - BSpl.UKnot(nbUKnots - 1));

                newUKnots[1] = oldUKnots[1] - a;
                newUKnots[nbUKnots + 2] = oldUKnots[nbUKnots] + a;
                newUMults[1] = newUMults[nbUKnots + 2] = 1;
                for (int i = 2; i <= nbUKnots + 1; i++)
                {
                    newUKnots[i] = oldUKnots[i - 1];
                    newUMults[i] = oldUMults[i - 1];
                }
                newUMults[2] = newUMults[nbUKnots + 1] = BSpl.UDegree();
                Geom_BSplineSurface res = new Geom_BSplineSurface(oldPoles,
                                                                  oldWeights,
                                                                  newUKnots,
                                                                  oldVKnots,
                                                                  newUMults,
                                                                  oldVMults,
                                                                  BSpl.UDegree(),
                                                                  BSpl.VDegree(),
                                                                  BSpl.IsUPeriodic(),
                                                                  BSpl.IsVPeriodic());
                BSpl = res;
            }
            else if (BSpl.UMultiplicity(1) > BSpl.UDegree()
                     || BSpl.UMultiplicity(BSpl.NbUKnots()) > BSpl.UDegree() + 1)
            {
                set = false;
            }
            if (set)
            {
                // OCCT.NET: OCCT makes the caller's surface periodic in place; a frozen surface is copied first
                // (decision 974)
                if (BSpl.IsFrozen())
                {
                    BSpl = (Geom_BSplineSurface)BSpl.Copy();
                }
                BSpl.SetUPeriodic(); // make periodic
                converted = true;
            }
        }

        if (vclosed && !BSpl.IsVPeriodic() && BSpl.NbVPoles() > 3)
        {
            bool set = true;
            // if degree+1 at ends, first change it to 1 by rearranging knots
            if (BSpl.VMultiplicity(1) == BSpl.VDegree() + 1
                && BSpl.VMultiplicity(BSpl.NbVKnots()) == BSpl.VDegree() + 1)
            {
                NCollection_Array2<gp_Pnt> oldPoles = BSpl.InternalPoles();
                NCollection_Array2<double> oldWeights = BSpl.InternalWeightsArray();
                int nbVKnots = BSpl.NbVKnots();
                NCollection_Array1<double> oldUKnots = BSpl.InternalUKnots();
                NCollection_Array1<double> oldVKnots = BSpl.InternalVKnots();
                NCollection_Array1<int> oldUMults = BSpl.InternalUMultiplicities();
                NCollection_Array1<int> oldVMults = BSpl.InternalVMultiplicities();

                NCollection_Array1<double> newVKnots = new(1, nbVKnots + 2);
                NCollection_Array1<int> newVMults = new(1, nbVKnots + 2);
                double a =
                    0.5 * (BSpl.VKnot(2) - BSpl.VKnot(1) + BSpl.VKnot(nbVKnots) - BSpl.VKnot(nbVKnots - 1));

                newVKnots[1] = oldVKnots[1] - a;
                newVKnots[nbVKnots + 2] = oldVKnots[nbVKnots] + a;
                newVMults[1] = newVMults[nbVKnots + 2] = 1;
                for (int i = 2; i <= nbVKnots + 1; i++)
                {
                    newVKnots[i] = oldVKnots[i - 1];
                    newVMults[i] = oldVMults[i - 1];
                }
                newVMults[2] = newVMults[nbVKnots + 1] = BSpl.VDegree();
                Geom_BSplineSurface res = new Geom_BSplineSurface(oldPoles,
                                                                  oldWeights,
                                                                  oldUKnots,
                                                                  newVKnots,
                                                                  oldUMults,
                                                                  newVMults,
                                                                  BSpl.UDegree(),
                                                                  BSpl.VDegree(),
                                                                  BSpl.IsUPeriodic(),
                                                                  BSpl.IsVPeriodic());
                BSpl = res;
            }
            else if (BSpl.VMultiplicity(1) > BSpl.VDegree()
                     || BSpl.VMultiplicity(BSpl.NbVKnots()) > BSpl.VDegree() + 1)
            {
                set = false;
            }
            if (set)
            {
                // OCCT.NET: as above (decision 974)
                if (BSpl.IsFrozen())
                {
                    BSpl = (Geom_BSplineSurface)BSpl.Copy();
                }
                BSpl.SetVPeriodic(); // make periodic
                converted = true;
            }
        }

        if (!converted)
        {
            return newSurf;
        }
        newSurf = BSpl;
        if (substitute)
        {
            mySurf = newSurf;
        }
        return newSurf;
    }
}
