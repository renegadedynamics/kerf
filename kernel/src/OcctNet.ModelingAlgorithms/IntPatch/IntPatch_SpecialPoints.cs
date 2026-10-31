// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_SpecialPoints.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_SpecialPoints.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GeomAbs_SurfaceType;
using static Occt.IntPatch_SpecPntType;

namespace Occt;

/// <summary>
/// Special points of intersection lines: the apex of a cone, the poles of a sphere, points on the seams of a quadric.
/// Finds them and adds them (with correct parameters) to walking lines.
/// </summary>
/// <remarks>
/// OCCT's static class. Its file-local function <c>FuncPreciseSeam</c> is a private nested class, and
/// <c>AdjustPointAndVertex</c>'s optional vertex pointer is an overload taking the vertex by <c>ref</c>. The
/// protected members (ProcessSphere, ProcessCone, GetTangentToIntLineForCone) are internal.
/// </remarks>
public static class IntPatch_SpecialPoints
{
    /// <summary>The function whose zero is an intersection point lying on a seam (isoline) of the quadric.</summary>
    private sealed class FuncPreciseSeam : math_FunctionSetWithDerivatives
    {
        private readonly Adaptor3d_Surface myQSurf;
        private readonly Adaptor3d_Surface myPSurf;

        // 1 for U-coordinate, 0 - for V one.
        private readonly int mySeamCoordInd;

        // Constant parameter of iso-line
        private readonly double myIsoParameter;

        public FuncPreciseSeam(Adaptor3d_Surface theQSurf, Adaptor3d_Surface thePSurf, bool isTheUSeam,
                               double theIsoParameter)
        {
            myQSurf = theQSurf;
            myPSurf = thePSurf;
            mySeamCoordInd = isTheUSeam ? 1 : 0; // Defines, U- or V-seam is used
            myIsoParameter = theIsoParameter;
        }

        public override int NbVariables() => 3;

        public override int NbEquations() => 3;

        public override bool Value(math_Vector theX, math_Vector theF)
        {
            try
            {
                int anIndX = theX.Lower(), anIndF = theF.Lower();
                Span<double> aUV = [myIsoParameter, myIsoParameter];
                aUV[mySeamCoordInd] = theX[anIndX + 2];
                gp_Pnt aP1 = myPSurf.Value(theX[anIndX], theX[anIndX + 1]);
                gp_Pnt aP2 = myQSurf.Value(aUV[0], aUV[1]);

                (aP1.XYZ() - aP2.XYZ()).Coord(out theF[anIndF], out theF[anIndF + 1], out theF[anIndF + 2]);
            }
            catch (Standard_Failure)
            {
                return false;
            }

            return true;
        }

        public override bool Derivatives(math_Vector theX, math_Matrix theD)
        {
            try
            {
                int anIndX = theX.Lower(), anIndRD = theD.LowerRow(), anIndCD = theD.LowerCol();
                Span<double> aUV = [myIsoParameter, myIsoParameter];
                aUV[mySeamCoordInd] = theX[anIndX + 2];

                // 0 for U-coordinate, 1 - for V one
                myPSurf.D1(theX[anIndX], theX[anIndX + 1], out _, out gp_Vec aD10, out gp_Vec aD11);
                myQSurf.D1(aUV[0], aUV[1], out _, out gp_Vec aD20, out gp_Vec aD21);

                // d/dX1
                aD10.Coord(out theD[anIndRD, anIndCD], out theD[anIndRD + 1, anIndCD], out theD[anIndRD + 2, anIndCD]);

                // d/dX2
                aD11.Coord(out theD[anIndRD, anIndCD + 1], out theD[anIndRD + 1, anIndCD + 1],
                           out theD[anIndRD + 2, anIndCD + 1]);

                // d/dX3
                (mySeamCoordInd == 0 ? aD20 : aD21).Reversed().Coord(out theD[anIndRD, anIndCD + 2],
                                                                    out theD[anIndRD + 1, anIndCD + 2],
                                                                    out theD[anIndRD + 2, anIndCD + 2]);
            }
            catch (Standard_Failure)
            {
                return false;
            }

            return true;
        }

        public override bool Values(math_Vector theX, math_Vector theF, math_Matrix theD)
        {
            if (!Value(theX, theF))
            {
                return false;
            }

            if (!Derivatives(theX, theD))
            {
                return false;
            }

            return true;
        }
    }

    // Computes tangent having the given parameter. See calling method(s) for detailed information.
    private static gp_XYZ GetTangent(double theConeSemiAngle, double theParameter)
    {
        double aW2 = theParameter * theParameter;
        double aCosUn = (1.0 - aW2) / (1.0 + aW2);
        double aSinUn = 2.0 * theParameter / (1.0 + aW2);

        double aTanA = Math.Tan(theConeSemiAngle);
        return new gp_XYZ(aTanA * aCosUn, aTanA * aSinUn, 1.0);
    }

    // Checks if thePt is in theSurf (with given tolerance). Returns the foot of projection (theProjPt) and its
    // parameters on theSurf.
    private static bool IsPointOnSurface(Adaptor3d_Surface theSurf, in gp_Pnt thePt, double theTol,
                                         ref gp_Pnt theProjPt, ref double theUpar, ref double theVpar)
    {
        bool aRetVal = false;

        switch (theSurf.GetType())
        {
            case GeomAbs_Plane:
            case GeomAbs_Cylinder:
            case GeomAbs_Cone:
            case GeomAbs_Sphere:
            case GeomAbs_Torus:
            case GeomAbs_SurfaceOfExtrusion:
            case GeomAbs_SurfaceOfRevolution:
            {
                Extrema_ExtPS anExtr = new(thePt, theSurf, theSurf.UResolution(theTol), theSurf.VResolution(theTol),
                                           Extrema_ExtFlag.Extrema_ExtFlag_MIN);
                if (!anExtr.IsDone() || (anExtr.NbExt() < 1))
                {
                    aRetVal = false;
                }
                else
                {
                    int anExtrIndex = 1;
                    double aSqDistMin = anExtr.SquareDistance(anExtrIndex);
                    for (int i = anExtrIndex + 1; i <= anExtr.NbExt(); i++)
                    {
                        double aSqD = anExtr.SquareDistance(i);
                        if (aSqD < aSqDistMin)
                        {
                            aSqDistMin = aSqD;
                            anExtrIndex = i;
                        }
                    }

                    if (aSqDistMin > theTol * theTol)
                    {
                        aRetVal = false;
                    }
                    else
                    {
                        theProjPt = new gp_Pnt(anExtr.Point(anExtrIndex).Value().XYZ());
                        anExtr.Point(anExtrIndex).Parameter(out theUpar, out theVpar);
                        aRetVal = true;
                    }
                }
            }
            break;
            default:
            {
                Extrema_GenLocateExtPS anExtr = new(theSurf);
                anExtr.Perform(thePt, theUpar, theVpar);
                if (!anExtr.IsDone() || (anExtr.SquareDistance() > theTol * theTol))
                {
                    aRetVal = false;
                }
                else
                {
                    anExtr.Point().Parameter(out theUpar, out theVpar);
                    theProjPt = new gp_Pnt(anExtr.Point().Value().XYZ());
                    aRetVal = true;
                }
            }
            break;
        }

        return aRetVal;
    }

    /// <summary>
    /// Computes the point at the intersection of the isolines U = 0 and V = 0 of <paramref name="theQSurf"/> lying on
    /// <paramref name="thePSurf"/>, adjusted to <paramref name="theRefPt"/>'s periods. If
    /// <paramref name="theIsReversed"/>, theQSurf is the second surface of the point.
    /// </summary>
    public static bool AddCrossUVIsoPoint(Adaptor3d_Surface theQSurf, Adaptor3d_Surface thePSurf,
                                          in IntSurf_PntOn2S theRefPt, double theTol,
                                          ref IntSurf_PntOn2S theAddedPoint, bool theIsReversed = false)
    {
        Span<double> anArrOfPeriod = [0.0, 0.0, 0.0, 0.0];
        IntSurf.SetPeriod(theIsReversed ? thePSurf : theQSurf, theIsReversed ? theQSurf : thePSurf, anArrOfPeriod);

        // Not quadric point
        double aU0, aV0;
        if (theIsReversed)
        {
            theRefPt.ParametersOnS1(out aU0, out aV0);
        }
        else
        {
            theRefPt.ParametersOnS2(out aU0, out aV0);
        }

        // Quadric point
        double aUquad = 0.0, aVquad = 0.0;

        theQSurf.D0(aUquad, aVquad, out gp_Pnt aPQuad);

        Extrema_GenLocateExtPS anExtr = new(thePSurf);
        anExtr.Perform(aPQuad, aU0, aV0);

        if (!anExtr.IsDone())
        {
            return false;
        }

        if (anExtr.SquareDistance() > theTol * theTol)
        {
            return false;
        }

        anExtr.Point().Parameter(out aU0, out aV0);
        gp_Pnt aP0 = anExtr.Point().Value();

        if (theIsReversed)
        {
            theAddedPoint.SetValue(new gp_Pnt(0.5 * (aP0.XYZ() + aPQuad.XYZ())), aU0, aV0, aUquad, aVquad);
        }
        else
        {
            theAddedPoint.SetValue(new gp_Pnt(0.5 * (aP0.XYZ() + aPQuad.XYZ())), aUquad, aVquad, aU0, aV0);
        }

        AdjustPointAndVertex(theRefPt, anArrOfPeriod, ref theAddedPoint);

        return true;
    }

    /// <summary>
    /// Computes the point lying strictly on the isoline U = <paramref name="theIsoParameter"/> (if
    /// <paramref name="theIsU"/>) or V = theIsoParameter of <paramref name="theQSurf"/> and on
    /// <paramref name="thePSurf"/>. The vectors hold the U and V parameters on thePSurf, then the free parameter on
    /// theQSurf.
    /// </summary>
    public static bool AddPointOnUorVIso(Adaptor3d_Surface theQSurf, Adaptor3d_Surface thePSurf,
                                         in IntSurf_PntOn2S theRefPt, bool theIsU, double theIsoParameter,
                                         math_Vector theToler, math_Vector theInitPoint, math_Vector theInfBound,
                                         math_Vector theSupBound, ref IntSurf_PntOn2S theAddedPoint,
                                         bool theIsReversed = false)
    {
        Span<double> anArrOfPeriod = [0.0, 0.0, 0.0, 0.0];
        IntSurf.SetPeriod(theIsReversed ? thePSurf : theQSurf, theIsReversed ? theQSurf : thePSurf, anArrOfPeriod);

        FuncPreciseSeam aF = new(theQSurf, thePSurf, theIsU, theIsoParameter);

        math_FunctionSetRoot aSRF = new(aF, theToler);
        aSRF.Perform(aF, theInitPoint, theInfBound, theSupBound);

        if (!aSRF.IsDone())
        {
            return false;
        }

        math_Vector aRoots = new(theInitPoint.Lower(), theInitPoint.Upper());
        aSRF.Root(aRoots);

        // On parametric
        double aU0 = aRoots[1], aV0 = aRoots[2];

        // On quadric
        double aUquad = theIsU ? 0.0 : aRoots[3];
        double aVquad = theIsU ? aRoots[3] : 0.0;
        gp_Pnt aPQuad = theQSurf.Value(aUquad, aVquad);
        gp_Pnt aP0 = thePSurf.Value(aU0, aV0);

        if (theIsReversed)
        {
            theAddedPoint.SetValue(new gp_Pnt(0.5 * (aP0.XYZ() + aPQuad.XYZ())), aU0, aV0, aUquad, aVquad);
        }
        else
        {
            theAddedPoint.SetValue(new gp_Pnt(0.5 * (aP0.XYZ() + aPQuad.XYZ())), aUquad, aVquad, aU0, aV0);
        }

        AdjustPointAndVertex(theRefPt, anArrOfPeriod, ref theAddedPoint);
        return true;
    }

    /// <summary>
    /// Computes the "special point" on a sphere (the U parameter at a pole), in [0, 2*PI]; the caller adjusts it.
    /// </summary>
    internal static bool ProcessSphere(in IntSurf_PntOn2S thePtIso, in gp_Vec theDUofPSurf, in gp_Vec theDVofPSurf,
                                       bool theIsReversed, double theVquad, ref double theUquad,
                                       out bool theIsIsoChoosen)
    {
        theIsIsoChoosen = false;

        // Vector with {cos(Uq), sin(Uq)} coordinates. It is always normalized.
        gp_Vec2d aV1 = new();

        if ((Math.Abs(theDUofPSurf.Z()) < Precision.PConfusion()) && (Math.Abs(theDVofPSurf.Z()) < Precision.PConfusion()))
        {
            // Example of this case is an intersection of a plane with a sphere when the plane tangents the sphere in
            // some pole (i.e. only one intersection point, not line). In this case, U-coordinate of the sphere is
            // undefined (can be really anything). Another reason is that we have tangent zone around the pole (see bug
            // #26576). Computation of correct value of theUquad is impossible. Therefore, (in order to return
            // something) we will consider the intersection line goes along some isoline in neighborhood of the pole.
            double aUIso, aVIso;
            if (theIsReversed)
            {
                thePtIso.ParametersOnS2(out aUIso, out aVIso);
            }
            else
            {
                thePtIso.ParametersOnS1(out aUIso, out aVIso);
            }

            theUquad = aUIso;
            theIsIsoChoosen = true;
        }
        else
        {
            if (Math.Abs(theDUofPSurf.Z()) > Math.Abs(theDVofPSurf.Z()))
            {
                double aDusDvs = theDVofPSurf.Z() / theDUofPSurf.Z();
                aV1.SetCoord(theDUofPSurf.X() * aDusDvs - theDVofPSurf.X(), theDUofPSurf.Y() * aDusDvs - theDVofPSurf.Y());
            }
            else
            {
                double aDvsDus = theDUofPSurf.Z() / theDVofPSurf.Z();
                aV1.SetCoord(theDVofPSurf.X() * aDvsDus - theDUofPSurf.X(), theDVofPSurf.Y() * aDvsDus - theDUofPSurf.Y());
            }

            aV1.Normalize();

            if (Math.Abs(aV1.X()) > Math.Abs(aV1.Y()))
            {
                theUquad = Math.CopySign(Math.Asin(aV1.Y()), theVquad);
            }
            else
            {
                theUquad = Math.CopySign(Math.Acos(aV1.X()), theVquad);
            }
        }

        return true;
    }

    /// <summary>
    /// Computes the "special point" on a cone (the U parameter at the apex), in [0, 2*PI]; the caller adjusts it.
    /// </summary>
    internal static bool ProcessCone(in IntSurf_PntOn2S thePtIso, in gp_Vec theDUofPSurf, in gp_Vec theDVofPSurf,
                                     in gp_Cone theCone, bool theIsReversed, ref double theUquad,
                                     out bool theIsIsoChoosen)
    {
        theIsIsoChoosen = false;

        // A plane tangent to 2nd (intersected) surface. Its normal.
        gp_XYZ aTgPlaneZ = theDUofPSurf.Crossed(theDVofPSurf).XYZ();
        double aSqModTg = aTgPlaneZ.SquareModulus();
        if (aSqModTg < Precision.SquareConfusion())
        {
            theIsIsoChoosen = true;
        }

        gp_XYZ[] aTgILine = new gp_XYZ[2];
        int aNbTangent = !theIsIsoChoosen
                             ? GetTangentToIntLineForCone(theCone.SemiAngle(), aTgPlaneZ.Divided(Math.Sqrt(aSqModTg)),
                                                          aTgILine)
                             : 0;

        if (aNbTangent == 0)
        {
            theIsIsoChoosen = true;
        }
        else
        {
            double aPeriod = M_PI + M_PI;
            double aUIso, aVIso;
            if (theIsReversed)
            {
                thePtIso.ParametersOnS2(out aUIso, out aVIso);
            }
            else
            {
                thePtIso.ParametersOnS1(out aUIso, out aVIso);
            }

            aUIso = ElCLib.InPeriod(aUIso, 0.0, aPeriod);

            // Sought U-parameter in the apex point

            // For 2 possible parameters value, one will be chosen which is nearer to aUIso.
            double aMinDelta = RealLast();
            for (int anIdx = 0; anIdx < aNbTangent; anIdx++)
            {
                // Vector {cos(a), sin(a)}
                gp_Vec2d aVecCS = new(aTgILine[anIdx].X(), aTgILine[anIdx].Y());
                double aSqMod = aVecCS.SquareMagnitude();
                if (aSqMod < Precision.SquareConfusion())
                {
                    theIsIsoChoosen = true;
                    break;
                }

                // Normalize
                aVecCS.Divide(Math.Sqrt(aSqMod));

                // Angle in range [0, PI/2]
                double anUq = (Math.Abs(aVecCS.X()) < Math.Abs(aVecCS.Y())) ? Math.Acos(Math.Abs(aVecCS.X()))
                                                                            : Math.Asin(Math.Abs(aVecCS.Y()));

                // Convert angles to the range [0, 2*PI]
                if (aVecCS.Y() < 0.0)
                {
                    if (aVecCS.X() > 0.0)
                    {
                        anUq = -anUq;
                    }
                    else
                    {
                        anUq += M_PI;
                    }
                }
                else if (aVecCS.X() < 0.0)
                {
                    anUq = M_PI - anUq;
                }

                // Select the parameter the nearest to aUIso
                anUq = ElCLib.InPeriod(anUq, 0.0, aPeriod);
                double aDelta = Math.Abs(anUq - aUIso);
                if (aDelta > M_PI)
                {
                    aDelta = aPeriod - aDelta;
                }

                if (aDelta < aMinDelta)
                {
                    aMinDelta = aDelta;
                    theUquad = anUq;
                }
            }
        }

        if (theIsIsoChoosen)
        {
            theIsIsoChoosen = true;

            double aUIso, aVIso;
            if (theIsReversed)
            {
                thePtIso.ParametersOnS2(out aUIso, out aVIso);
            }
            else
            {
                thePtIso.ParametersOnS1(out aUIso, out aVIso);
            }

            theUquad = aUIso;
            return true;
        }
        else
        {
            return true;
        }
    }

    /// <summary>
    /// The tangents (at most two, in <paramref name="theResult"/>) at the apex of a canonical cone of the
    /// intersection with the plane through the apex of normal <paramref name="thePlnNormal"/> (normalized). Returns
    /// their number.
    /// </summary>
    internal static int GetTangentToIntLineForCone(double theConeSemiAngle, in gp_XYZ thePlnNormal, gp_XYZ[] theResult)
    {
        double aNullTol = Epsilon(1.0);
        double aTanA = Math.Tan(theConeSemiAngle);
        double aA = thePlnNormal.Z() / aTanA - thePlnNormal.X();
        double aB = thePlnNormal.Y();
        double aC = thePlnNormal.Z() / aTanA + thePlnNormal.X();

        if (Math.Abs(aA) < aNullTol)
        {
            if (Math.Abs(aB) > aNullTol)
            {
                // The plane goes along the cone generatrix.
                theResult[0] = GetTangent(theConeSemiAngle, -aC / (aB + aB));
                return 1;
            }

            // The cone and the plane have only one common point. It is the cone apex.
            return 0;
        }

        // Discriminant of this equation is equal to
        double aDiscr = thePlnNormal.Z() / Math.Sin(theConeSemiAngle);
        aDiscr = 1.0 - aDiscr * aDiscr;

        if (Math.Abs(aDiscr) < aNullTol)
        {
            // The plane goes along the cone generatrix. Mathematically, this condition is equivalent to the one above
            // (|aA| < aNullTol and |aB| > aNullTol), but is separated for numerical stability.
            theResult[0] = GetTangent(theConeSemiAngle, -aB / aA);
            return 1;
        }
        else if (aDiscr > 0.0)
        {
            double aRD = Math.Sqrt(aDiscr);
            theResult[0] = GetTangent(theConeSemiAngle, (-aB + aRD) / aA);
            theResult[1] = GetTangent(theConeSemiAngle, (-aB - aRD) / aA);
            return 2;
        }

        // We will never come here.
        return 0;
    }

    /// <summary>
    /// Computes the pole of a sphere or the apex of a cone (<paramref name="theQSurf"/>) as an intersection point with
    /// <paramref name="thePSurf"/>, in <paramref name="theAddedPoint"/> (not added to any line).
    /// <paramref name="thePtIso"/> gives the isoline on which the pole is placed; <paramref name="theVertex"/> must be
    /// initialized (it may be updated). If <paramref name="theIsReqRefCheck"/>, the vertex must be the pole. Throws
    /// Standard_TypeMismatch for other quadrics.
    /// </summary>
    public static bool AddSingularPole(Adaptor3d_Surface theQSurf, Adaptor3d_Surface thePSurf,
                                       in IntSurf_PntOn2S thePtIso, ref IntPatch_Point theVertex,
                                       ref IntSurf_PntOn2S theAddedPoint, bool theIsReversed = false,
                                       bool theIsReqRefCheck = false)
    {
        // On parametric
        double aU0, aV0;
        // aPQuad is Pole
        gp_Pnt aP0 = new();
        double aUquad, aVquad;
        if (theIsReversed)
        {
            theVertex.Parameters(out aU0, out aV0, out aUquad, out aVquad);
        }
        else
        {
            theVertex.Parameters(out aUquad, out aVquad, out aU0, out aV0);
        }

        aUquad = 0.0;

        if (theQSurf.GetType() == GeomAbs_Sphere)
        {
            aVquad = Math.CopySign(M_PI_2, aVquad);
        }
        else if (theQSurf.GetType() == GeomAbs_Cone)
        {
            gp_Cone aCo = theQSurf.Cone();
            double aRadius = aCo.RefRadius();
            double aSemiAngle = aCo.SemiAngle();
            aVquad = -aRadius / Math.Sin(aSemiAngle);
        }
        else
        {
            throw new Standard_TypeMismatch("IntPatch_SpecialPoints::AddSingularPole(),Unsupported quadric with Pole");
        }

        theQSurf.D0(aUquad, aVquad, out gp_Pnt aPQuad);
        double aTol = theVertex.Tolerance();
        if (theIsReqRefCheck && (aPQuad.SquareDistance(theVertex.Value()) >= aTol * aTol))
        {
            return false;
        }

        if (!IsPointOnSurface(thePSurf, aPQuad, aTol, ref aP0, ref aU0, ref aV0))
        {
            return false;
        }

        // Pole is an intersection point (lies in the quadric and the parametric surface)

        if (theIsReversed)
        {
            theAddedPoint.SetValue(new gp_Pnt(0.5 * (aP0.XYZ() + aPQuad.XYZ())), aU0, aV0, aUquad, aVquad);
        }
        else
        {
            theAddedPoint.SetValue(new gp_Pnt(0.5 * (aP0.XYZ() + aPQuad.XYZ())), aUquad, aVquad, aU0, aV0);
        }

        bool isSame = theAddedPoint.IsSame(theVertex.PntOn2S(), Precision.Confusion());

        // Found pole does not exist in the Walking-line. It must be added there (with correct 2D-parameters).

        // 2D-parameters of thePSurf surface have already been found (aU0, aV0). Let find 2D-parameters on the quadric.
        // The algorithm depends on the type of the quadric (sphere or cone). The derivatives of thePSurf are
        // transformed into the coordinate system of the quadric.

        thePSurf.D1(aU0, aV0, out _, out gp_Vec aVecDu, out gp_Vec aVecDv);

        // Transforms parametric surface in coordinate-system of the quadric
        gp_Trsf aTr = new();
        aTr.SetTransformation((theQSurf.GetType() == GeomAbs_Sphere) ? theQSurf.Sphere().Position()
                                                                      : theQSurf.Cone().Position());

        // Derivatives of transformed thePSurf
        aVecDu.Transform(aTr);
        aVecDv.Transform(aTr);

        bool isIsoChoosen;

        if (theQSurf.GetType() == GeomAbs_Sphere)
        {
            if (!ProcessSphere(thePtIso, aVecDu, aVecDv, theIsReversed, aVquad, ref aUquad, out isIsoChoosen))
            {
                return false;
            }
        }
        else // if(theQSurf->GetType() == GeomAbs_Cone)
        {
            if (!ProcessCone(thePtIso, aVecDu, aVecDv, theQSurf.Cone(), theIsReversed, ref aUquad, out isIsoChoosen))
            {
                return false;
            }
        }

        if (theIsReversed)
        {
            theAddedPoint.SetValue(new gp_Pnt(0.5 * (aP0.XYZ() + aPQuad.XYZ())), aU0, aV0, aUquad, aVquad);
        }
        else
        {
            theAddedPoint.SetValue(new gp_Pnt(0.5 * (aP0.XYZ() + aPQuad.XYZ())), aUquad, aVquad, aU0, aV0);
        }

        if (isSame)
        {
            theVertex.SetValue(theAddedPoint);
            return true;
        }

        if (!isIsoChoosen)
        {
            Span<double> anArrOfPeriod = stackalloc double[4];
            if (theIsReversed)
            {
                IntSurf.SetPeriod(thePSurf, theQSurf, anArrOfPeriod);
            }
            else
            {
                IntSurf.SetPeriod(theQSurf, thePSurf, anArrOfPeriod);
            }

            AdjustPointAndVertex(theVertex.PntOn2S(), anArrOfPeriod, ref theAddedPoint);
        }
        else
        {
            theVertex.SetValue(theAddedPoint);
        }

        return true;
    }

    /// <summary>
    /// After a special point added to a line: the point (<paramref name="theNewPoint"/>, in and out) with which the
    /// line continues or a new line starts, adjusted to <paramref name="theRefPt"/>.
    /// </summary>
    public static bool ContinueAfterSpecialPoint(Adaptor3d_Surface theQSurf, Adaptor3d_Surface thePSurf,
                                                 in IntSurf_PntOn2S theRefPt, IntPatch_SpecPntType theSPType,
                                                 double theTol2D, ref IntSurf_PntOn2S theNewPoint,
                                                 bool theIsReversed = false)
    {
        if (theSPType == IntPatch_SPntNone)
        {
            return false;
        }

        if (theNewPoint.IsSame(theRefPt, Precision.Confusion(), theTol2D))
        {
            return false;
        }

        if ((theSPType == IntPatch_SPntPole) && (theQSurf.GetType() == GeomAbs_Cone))
        {
            // Check if the condition b) is satisfied. Repeat the same steps as in AddSingularPole.

            // On parametric
            double aU0, aV0;
            // On quadric
            double aUquad, aVquad;

            if (theIsReversed)
            {
                theNewPoint.Parameters(out aU0, out aV0, out aUquad, out aVquad);
            }
            else
            {
                theNewPoint.Parameters(out aUquad, out aVquad, out aU0, out aV0);
            }

            thePSurf.D1(aU0, aV0, out _, out gp_Vec aVecDu, out gp_Vec aVecDv);

            // Transforms parametric surface in coordinate-system of the quadric
            gp_Trsf aTr = new();
            aTr.SetTransformation(theQSurf.Cone().Position());

            // Derivatives of transformed thePSurf
            aVecDu.Transform(aTr);
            aVecDv.Transform(aTr);

            ProcessCone(theRefPt, aVecDu, aVecDv, theQSurf.Cone(), theIsReversed, ref aUquad, out _);

            theNewPoint.SetValue(!theIsReversed, aUquad, aVquad);
        }

        // In case of going through the Pole/Apex, U-parameter of the quadric surface will change to +/-PI, with
        // exceptions (C0 second surface; huge curvature): so the U-parameter of the quadric is adjusted with period
        // PI/2 instead of 2PI, forbidding "jumps" between two neighbor Walking-points greater than PI/4.

        double aPeriod = (theSPType == IntPatch_SPntPole) ? M_PI_2 : 2.0 * M_PI;

        double aUpPeriod = thePSurf.IsUPeriodic() ? thePSurf.UPeriod() : 0.0;
        double aUqPeriod = theQSurf.IsUPeriodic() ? aPeriod : 0.0;
        double aVpPeriod = thePSurf.IsVPeriodic() ? thePSurf.VPeriod() : 0.0;
        double aVqPeriod = theQSurf.IsVPeriodic() ? aPeriod : 0.0;

        Span<double> anArrOfPeriod =
        [
            theIsReversed ? aUpPeriod : aUqPeriod,
            theIsReversed ? aVpPeriod : aVqPeriod,
            theIsReversed ? aUqPeriod : aUpPeriod,
            theIsReversed ? aVqPeriod : aVpPeriod,
        ];

        AdjustPointAndVertex(theRefPt, anArrOfPeriod, ref theNewPoint);
        return true;
    }

    /// <summary>
    /// Sets <paramref name="theNewPoint"/>'s parameters closest to <paramref name="theRefPoint"/>'s by adding or
    /// subtracting the periods (U and V of the first surface, then of the second; 0 where not periodic).
    /// </summary>
    public static void AdjustPointAndVertex(in IntSurf_PntOn2S theRefPoint, ReadOnlySpan<double> theArrPeriods,
                                            ref IntSurf_PntOn2S theNewPoint)
    {
        Adjust(theRefPoint, theArrPeriods, ref theNewPoint, out _);
    }

    /// <summary>
    /// As <see cref="AdjustPointAndVertex(in IntSurf_PntOn2S, ReadOnlySpan{double}, ref IntSurf_PntOn2S)"/>, and sets
    /// <paramref name="theVertex"/>'s parameters as the new point's (OCCT's non-null vertex pointer).
    /// </summary>
    public static void AdjustPointAndVertex(in IntSurf_PntOn2S theRefPoint, ReadOnlySpan<double> theArrPeriods,
                                            ref IntSurf_PntOn2S theNewPoint, ref IntPatch_Point theVertex)
    {
        Adjust(theRefPoint, theArrPeriods, ref theNewPoint, out (double, double, double, double) aPar);
        theVertex.SetParameters(aPar.Item1, aPar.Item2, aPar.Item3, aPar.Item4);
    }

    private static void Adjust(in IntSurf_PntOn2S theRefPoint, ReadOnlySpan<double> theArrPeriods,
                               ref IntSurf_PntOn2S theNewPoint, out (double, double, double, double) theParams)
    {
        Span<double> aRefPar = [0.0, 0.0];
        Span<double> aPar = [0.0, 0.0, 0.0, 0.0];
        theNewPoint.Parameters(out aPar[0], out aPar[1], out aPar[2], out aPar[3]);

        for (int i = 0; i < 4; i++)
        {
            if (theArrPeriods[i] == 0)
            {
                continue;
            }

            double aPeriod = theArrPeriods[i], aHalfPeriod = 0.5 * theArrPeriods[i];

            if (i < 2)
            { // 1st surface is used
                theRefPoint.ParametersOnS1(out aRefPar[0], out aRefPar[1]);
            }
            else
            {
                theRefPoint.ParametersOnS2(out aRefPar[0], out aRefPar[1]);
            }

            int aRefInd = i % 2;

            {
                double aDeltaPar = aRefPar[aRefInd] - aPar[i];
                double anIncr = Math.CopySign(aPeriod, aDeltaPar);
                while ((aDeltaPar > aHalfPeriod) || (aDeltaPar < -aHalfPeriod))
                {
                    aPar[i] += anIncr;
                    aDeltaPar = aRefPar[aRefInd] - aPar[i];
                }
            }
        }

        theParams = (aPar[0], aPar[1], aPar[2], aPar[3]);
        theNewPoint.SetValue(aPar[0], aPar[1], aPar[2], aPar[3]);
    }
}
