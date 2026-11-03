// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_InterferencePolyhedron.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_InterferencePolyhedron.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using Kerf.Geometry;
using static Occt.Intf_PIType;

namespace Occt;

/// <summary>
/// The interference of two polyhedra (or of a polyhedron with itself): the section lines and tangent zones of their
/// triangles, found by intersecting the pairs of triangles whose boxes overlap (an IntPatch_PolyhedronBVH traversal).
/// </summary>
/// <remarks>
/// A working object (decision 21). The pairs are intersected in the traversal's order, which the chaining of the
/// section lines depends on. OCCT's <c>Perform(Obje)</c> calls a self-interference <c>Interference(Obje)</c> whose
/// body is empty, so it finds nothing, where the constructor of the self-interference computes it
/// (occt-issues.md, 5D). TangentZoneValue's debug print of the points it couldn't insert (Intf_SectionPoint's Dump,
/// which prints only in a debug build) is left out.
/// </remarks>
public class IntPatch_InterferencePolyhedron : Intf_Interference
{
    private static readonly int[] Pourcent3 = [0, 1, 2, 0, 1, 2, 0, 1, 2];

    private readonly int[] OI = new int[3];
    private readonly int[] TI = new int[3];
    private readonly double[,] dpOpT = new double[3, 3];
    private readonly double[,] dpOeT = new double[3, 3];
    private readonly double[,] deOpT = new double[3, 3];
    private readonly Xyz[] voo = new Xyz[3];
    private readonly Xyz[] vtt = new Xyz[3];
    private double Incidence;

    /// <summary>An empty interference of polyhedra.</summary>
    public IntPatch_InterferencePolyhedron()
        : base(false)
    {
        Incidence = 0;
    }

    /// <summary>Computes the interference of the two polyhedra.</summary>
    public IntPatch_InterferencePolyhedron(IntPatch_Polyhedron FirstPol, IntPatch_Polyhedron SeconPol)
        : base(false)
    {
        Incidence = 0;
        if (!IntPatch_PolyhedronTool.Bounding(FirstPol).IsOut(IntPatch_PolyhedronTool.Bounding(SeconPol)))
        {
            Tolerance = IntPatch_PolyhedronTool.DeflectionOverEstimation(FirstPol)
                        + IntPatch_PolyhedronTool.DeflectionOverEstimation(SeconPol);
            if (Tolerance == 0.0)
            {
                Tolerance = Epsilon(1000.0);
            }
            Interference(FirstPol, SeconPol);
        }
    }

    /// <summary>Computes the self-interference of the polyhedron.</summary>
    public IntPatch_InterferencePolyhedron(IntPatch_Polyhedron Objet)
        : base(true)
    {
        Incidence = 0;
        Tolerance = IntPatch_PolyhedronTool.DeflectionOverEstimation(Objet) * 2;
        if (Tolerance == 0.0)
        {
            Tolerance = Epsilon(1000.0);
        }
        Interference(Objet, Objet); //-- lbr le 5 juillet 96
    }

    /// <summary>Computes the interference of the two polyhedra.</summary>
    public void Perform(IntPatch_Polyhedron FirstPol, IntPatch_Polyhedron SeconPol)
    {
        SelfInterference(false);
        if (!IntPatch_PolyhedronTool.Bounding(FirstPol).IsOut(IntPatch_PolyhedronTool.Bounding(SeconPol)))
        {
            Tolerance = IntPatch_PolyhedronTool.DeflectionOverEstimation(FirstPol)
                        + IntPatch_PolyhedronTool.DeflectionOverEstimation(SeconPol);
            if (Tolerance == 0.0)
            {
                Tolerance = Epsilon(1000.0);
            }
            Interference(FirstPol, SeconPol);
        }
    }

    /// <summary>
    /// Sets the self-interference mode and its tolerance, and computes nothing: OCCT's self-interference
    /// <c>Interference(Obje)</c> is empty (see the remarks).
    /// </summary>
    public void Perform(IntPatch_Polyhedron Objet)
    {
        SelfInterference(true);
        Tolerance = IntPatch_PolyhedronTool.DeflectionOverEstimation(Objet) * 2;
        if (Tolerance == 0.0)
        {
            Tolerance = Epsilon(1000.0);
        }
        Interference(Objet); // OCCT: an empty body (occt-issues.md, 5D)
    }

    private static void Interference(IntPatch_Polyhedron _)
    {
    }

    private void Interference(IntPatch_Polyhedron theFirstPol, IntPatch_Polyhedron theSecondPol)
    {
        // Build BVH sets for both polyhedra
        IntPatch_PolyhedronBVH aSet1 = new(theFirstPol);
        IntPatch_PolyhedronBVH aSet2 = new(theSecondPol);

        // Find candidate triangle pairs via BVH traversal
        IntPatch_BVHTraversal aTraversal = new();
        aTraversal.Perform(aSet1, aSet2, SelfIntf);

        // Process each candidate pair
        NCollection_DynamicArray<IntPatch_BVHTraversal.TrianglePair> aPairs = aTraversal.Pairs();
        foreach (IntPatch_BVHTraversal.TrianglePair aPair in aPairs)
        {
            Intersect(aPair.First, theFirstPol, aPair.Second, theSecondPol);
        }
    }

    //=======================================================================
    // function : Intersect
    // purpose  : Intersection of two triangles issue from two Polyhedron.
    //=======================================================================
    private void Intersect(int Tri1, IntPatch_Polyhedron FirstPol, int Tri2, IntPatch_Polyhedron SeconPol)
    {
        IntPatch_PolyhedronTool.Triangle(FirstPol, Tri1, out OI[0], out OI[1], out OI[2]);
        IntPatch_PolyhedronTool.Triangle(SeconPol, Tri2, out TI[0], out TI[1], out TI[2]);

        // If there is an intersection of a polyhedron with itself, the
        // intersections are excluded
        // from a triangle with connected triangles :

        if (SelfIntf)
        {
            if (OI[0] == TI[0] || OI[0] == TI[1] || OI[0] == TI[2] || OI[1] == TI[0] || OI[1] == TI[1]
                || OI[1] == TI[2] || OI[2] == TI[0] || OI[2] == TI[1] || OI[2] == TI[2])
            {
                return;
            }
        }

        // The precision of intersections includes two values ;

        // - Tolerance :This value allows detecting potential
        //              intersections in all cases.  The value should be the
        //            sum of upper bounds of tops pof two polyhedrons.

        // - floatGap : This value is the actual precision of calculation
        //              of line of section.Its value is very small, it
        //              allows having the same behaviour for
        //              geometry tests as for the values used.

        double floatGap = 1e-13; //-- Epsilon(1000.);

        // Equation of the triangle plane of the objet
        Intf.PlaneEquation(IntPatch_PolyhedronTool.Point(FirstPol, OI[0]),
                           IntPatch_PolyhedronTool.Point(FirstPol, OI[1]),
                           IntPatch_PolyhedronTool.Point(FirstPol, OI[2]),
                           out Xyz ONor, // Normal vector.
                           out double Odp); // Polar Distance.

        // Equation of the triangle plane of the tool
        Intf.PlaneEquation(IntPatch_PolyhedronTool.Point(SeconPol, TI[0]),
                           IntPatch_PolyhedronTool.Point(SeconPol, TI[1]),
                           IntPatch_PolyhedronTool.Point(SeconPol, TI[2]),
                           out Xyz TNor, // Normal vector.
                           out double Tdp); // Polar distance.

        // Scalar product of two normalized vectors -> cosinus of the angle
        Incidence = Math.Abs(TNor * ONor);

        // Distance of the plane of the triangle from the object by three points of SeconPol
        Span<double> dfOpT = stackalloc double[3];
        dfOpT[0] = ONor * (IntPatch_PolyhedronTool.Point(SeconPol, TI[0]).XYZ()) - Odp;
        dfOpT[1] = ONor * (IntPatch_PolyhedronTool.Point(SeconPol, TI[1]).XYZ()) - Odp;
        dfOpT[2] = ONor * (IntPatch_PolyhedronTool.Point(SeconPol, TI[2]).XYZ()) - Odp;

        // Distance of the plane of the triangle from the tool by three points of FirstPol
        Span<double> dpOfT = stackalloc double[3];
        dpOfT[0] = TNor * (IntPatch_PolyhedronTool.Point(FirstPol, OI[0]).XYZ()) - Tdp;
        dpOfT[1] = TNor * (IntPatch_PolyhedronTool.Point(FirstPol, OI[1]).XYZ()) - Tdp;
        dpOfT[2] = TNor * (IntPatch_PolyhedronTool.Point(FirstPol, OI[2]).XYZ()) - Tdp;

        // Values defining the couple of triangle dpOpT, dpOeT, deOpT
        CoupleCharacteristics(FirstPol, SeconPol);

        // If three  points  of the triangle of <SeconPol> are in the plane of the
        // triangle of <Obje> within <Tolerance> the eventual tangency zone is found.

        Intf_TangentZone TheTZ = new();
        if ((Math.Abs(dfOpT[0]) <= Tolerance && Math.Abs(dfOpT[1]) <= Tolerance && Math.Abs(dfOpT[2]) <= Tolerance)
            && (Math.Abs(dpOfT[0]) <= Tolerance && Math.Abs(dpOfT[1]) <= Tolerance
                && Math.Abs(dpOfT[2]) <= Tolerance)
            && (Math.Abs(dfOpT[0] + dfOpT[1] + dfOpT[2])
                != Math.Abs(dfOpT[0]) + Math.Abs(dfOpT[1]) + Math.Abs(dfOpT[2]))
            && (Math.Abs(dpOfT[0] + dpOfT[1] + dpOfT[2])
                != Math.Abs(dpOfT[0]) + Math.Abs(dpOfT[1]) + Math.Abs(dpOfT[2])))
        {
            if (TangentZoneValue(TheTZ, FirstPol, Tri1, SeconPol, Tri2))
            {
                if (!Insert(TheTZ))
                {
                    myTZones.Append(TheTZ);
                }
            }
        }

        // Otherwise line of section is calculated:
        else
        {
            int iObj, iToo;

            // Zone de stockage des resultats :
            int nbpiOT = 0;
            int nbpiO = 0;
            int nbpiT = 0;
            NCollection_Sequence<Intf_SectionPoint> piOT = new();
            Span<double> parO = stackalloc double[3];
            Span<double> parT = stackalloc double[3];

            // Indicateurs d arete touchee
            Span<int> edOT = stackalloc int[3];
            Span<int> edTT = stackalloc int[3];

            // Initializations
            parO[0] = parT[0] = parO[1] = parT[1] = parO[2] = parT[2] = -1.0;
            edOT[0] = edTT[0] = edOT[1] = edTT[1] = edOT[2] = edTT[2] = 1;

            // Singularite VERTEX VERTEX
            for (iObj = 0; iObj < 3; iObj++)
            {
                for (iToo = 0; iToo < 3; iToo++)
                {
                    if (dpOpT[iObj, iToo] <= floatGap)
                    {
                        piOT.Append(new Intf_SectionPoint(IntPatch_PolyhedronTool.Point(FirstPol, OI[iObj]),
                                                          Intf_VERTEX,
                                                          OI[iObj],
                                                          0,
                                                          0.0,
                                                          Intf_VERTEX,
                                                          TI[iToo],
                                                          0,
                                                          0.0,
                                                          Incidence));
                        parO[iObj] = parT[iToo] = 0.0;
                        edOT[Pourcent3[iObj + 2]] = edOT[iObj] = edTT[Pourcent3[iToo + 2]] = edTT[iToo] = 0;
                        nbpiOT++;
                        nbpiO++;
                        nbpiT++;
                    }
                }
            }

            // Singularite VERTEX EDGE
            int inext, jnext;
            for (iObj = 0; iObj < 3; iObj++)
            {
                if (parO[iObj] == -1.0)
                {
                    for (iToo = 0; iToo < 3; iToo++)
                    {
                        inext = Pourcent3[iToo + 1];
                        if (edTT[iToo] == 1)
                        {
                            if (dpOeT[iObj, iToo] <= floatGap && dpOeT[iObj, iToo] >= -floatGap)
                            {
                                if ((dpOpT[iObj, iToo] + dpOpT[iObj, inext]) < vtt[iToo].Modulus())
                                {
                                    parT[iToo] = dpOpT[iObj, iToo] / (dpOpT[iObj, iToo] + dpOpT[iObj, inext]);
                                    if (TI[iToo] > TI[inext])
                                    {
                                        parT[iToo] = 1.0 - parT[iToo];
                                    }
                                    piOT.Append(new Intf_SectionPoint(IntPatch_PolyhedronTool.Point(FirstPol, OI[iObj]),
                                                                      Intf_VERTEX,
                                                                      OI[iObj],
                                                                      0,
                                                                      0.0,
                                                                      Intf_EDGE,
                                                                      Math.Min(TI[iToo], TI[inext]),
                                                                      Math.Max(TI[iToo], TI[inext]),
                                                                      parT[iToo],
                                                                      Incidence));
                                    parO[iObj] = 0.0;
                                    edOT[Pourcent3[iObj + 2]] = 0;
                                    edOT[iObj] = 0;
                                    edTT[iToo] = 0;
                                    nbpiOT++;
                                    nbpiO++;
                                    nbpiT++;
                                }
                            }
                        }
                    }
                }
            }

            // Singularite EDGE VERTEX
            for (iToo = 0; iToo < 3; iToo++)
            {
                if (parT[iToo] == -1.0)
                {
                    for (iObj = 0; iObj < 3; iObj++)
                    {
                        inext = Pourcent3[iObj + 1];
                        if (edOT[iObj] == 1)
                        {
                            if (Math.Abs(deOpT[iObj, iToo]) <= floatGap)
                            {
                                if ((dpOpT[iObj, iToo] + dpOpT[inext, iToo]) < voo[iObj].Modulus())
                                {
                                    parO[iObj] = dpOpT[iObj, iToo] / (dpOpT[iObj, iToo] + dpOpT[inext, iToo]);
                                    if (OI[iObj] > OI[inext])
                                    {
                                        parO[iObj] = 1.0 - parO[iObj];
                                    }
                                    piOT.Append(new Intf_SectionPoint(IntPatch_PolyhedronTool.Point(SeconPol, TI[iToo]),
                                                                      Intf_EDGE,
                                                                      Math.Min(OI[iObj], OI[inext]),
                                                                      Math.Max(OI[iObj], OI[inext]),
                                                                      parO[iObj],
                                                                      Intf_VERTEX,
                                                                      TI[iToo],
                                                                      0,
                                                                      0.0,
                                                                      Incidence));
                                    parT[iToo] = 0.0;
                                    edOT[iObj] = edTT[Pourcent3[iToo + 2]] = edTT[iToo] = 0;
                                    nbpiOT++;
                                    nbpiO++;
                                    nbpiT++;
                                }
                            }
                        }
                    }
                }
            }

            // Singularite FACE VERTEX
            for (iToo = 0; iToo < 3; iToo++)
            {
                if (parT[iToo] != 0.0)
                {
                    if (Math.Abs(dfOpT[iToo]) <= floatGap)
                    {
                        piOT.Append(new Intf_SectionPoint(IntPatch_PolyhedronTool.Point(SeconPol, TI[iToo]),
                                                          Intf_FACE,
                                                          Tri1,
                                                          0,
                                                          0.0,
                                                          Intf_VERTEX,
                                                          TI[iToo],
                                                          0,
                                                          0.0,
                                                          Incidence));
                        parT[iToo] = 0.0;
                        edTT[Pourcent3[iToo + 2]] = edTT[iToo] = 0;
                        nbpiOT++;
                        nbpiT++;
                    }
                }
            }

            // Singularite VERTEX FACE
            for (iObj = 0; iObj < 3; iObj++)
            {
                if (parO[iObj] != 0.0)
                {
                    if (Math.Abs(dpOfT[iObj]) <= floatGap)
                    {
                        piOT.Append(new Intf_SectionPoint(IntPatch_PolyhedronTool.Point(FirstPol, OI[iObj]),
                                                          Intf_VERTEX,
                                                          OI[iObj],
                                                          0,
                                                          0.0,
                                                          Intf_FACE,
                                                          Tri2,
                                                          0,
                                                          0.0,
                                                          Incidence));
                        parO[iObj] = 0.0;
                        edOT[Pourcent3[iObj + 2]] = edOT[iObj] = 0;
                        nbpiOT++;
                        nbpiO++;
                    }
                }
            }

            // Singularite EDGE EDGE
            Point3D piO = new();
            Xyz piT = new();
            double lg;
            for (iObj = 0; iObj < 3; iObj++)
            {
                inext = Pourcent3[iObj + 1];
                if (edOT[iObj] == 1 && (dpOfT[iObj] * dpOfT[inext]) < 0.0)
                {
                    lg = dpOfT[iObj] / (dpOfT[iObj] - dpOfT[inext]);
                    if (lg > 0.0 && lg < 1.0)
                    {
                        for (iToo = 0; iToo < 3; iToo++)
                        {
                            jnext = Pourcent3[iToo + 1];
                            if (edTT[iToo] == 1 && (dfOpT[iToo] * dfOpT[jnext]) < 0.0)
                            {
                                lg = dfOpT[iToo] / (dfOpT[iToo] - dfOpT[jnext]);
                                if (lg > 0.0 && lg < 1.0)
                                {
                                    bool Pb = false;
                                    if (OI[iObj] > OI[inext])
                                    {
                                        double div = (dpOeT[inext, iToo] - dpOeT[iObj, iToo]);
                                        if (div > floatGap || div < -floatGap)
                                        {
                                            parO[iObj] = dpOeT[inext, iToo] / div;
                                            piO = new Point3D((IntPatch_PolyhedronTool.Point(FirstPol, OI[inext]).XYZ())
                                                             + (voo[iObj].Reversed() * parO[iObj]));
                                        }
                                        else
                                        {
                                            Pb = true;
                                        }
                                    }
                                    else
                                    {
                                        double div = dpOeT[iObj, iToo] - dpOeT[inext, iToo];
                                        if (div > floatGap || div < -floatGap)
                                        {
                                            parO[iObj] = dpOeT[iObj, iToo] / (dpOeT[iObj, iToo] - dpOeT[inext, iToo]);
                                            piO = new Point3D((IntPatch_PolyhedronTool.Point(FirstPol, OI[iObj]).XYZ())
                                                             + (voo[iObj] * parO[iObj]));
                                        }
                                        else
                                        {
                                            Pb = true;
                                        }
                                    }
                                    if (TI[iToo] > TI[jnext])
                                    {
                                        double div = (deOpT[iObj, jnext] - deOpT[iObj, iToo]);
                                        if (div > floatGap || div < -floatGap)
                                        {
                                            parT[iToo] = deOpT[iObj, jnext] / div;
                                            piT = (IntPatch_PolyhedronTool.Point(SeconPol, TI[jnext]).XYZ())
                                                  + (vtt[iToo].Reversed() * parT[iToo]);
                                        }
                                        else
                                        {
                                            Pb = true;
                                        }
                                    }
                                    else
                                    {
                                        double div = (deOpT[iObj, iToo] - deOpT[iObj, jnext]);
                                        if (div > floatGap || div < -floatGap)
                                        {
                                            parT[iToo] = deOpT[iObj, iToo] / div;
                                            piT = (IntPatch_PolyhedronTool.Point(SeconPol, TI[iToo]).XYZ())
                                                  + (vtt[iToo] * parT[iToo]);
                                        }
                                        else
                                        {
                                            Pb = true;
                                        }
                                    }
                                    if (!Pb)
                                    {
                                        piT -= piO.XYZ();
                                        lg = piT.Modulus();
                                        if (lg <= floatGap)
                                        {
                                            piOT.Append(new Intf_SectionPoint(piO,
                                                                              Intf_EDGE,
                                                                              Math.Min(OI[iObj], OI[inext]),
                                                                              Math.Max(OI[iObj], OI[inext]),
                                                                              parO[iObj],
                                                                              Intf_EDGE,
                                                                              Math.Min(TI[iToo], TI[jnext]),
                                                                              Math.Max(TI[iToo], TI[jnext]),
                                                                              parT[iToo],
                                                                              Incidence));
                                            edOT[iObj] = edTT[iToo] = 0;
                                            nbpiOT++;
                                            nbpiO++;
                                            nbpiT++;
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }

            // Intersection EDGE FACE
            for (iObj = 0; iObj < 3; iObj++)
            {
                inext = Pourcent3[iObj + 1];
                if (edOT[iObj] == 1 && (dpOfT[iObj] * dpOfT[inext]) < 0.0)
                {
                    lg = dpOfT[iObj] / (dpOfT[iObj] - dpOfT[inext]);
                    if (lg > 0.0 && lg < 1.0)
                    {
                        parO[iObj] = lg;
                        piO = new Point3D((IntPatch_PolyhedronTool.Point(FirstPol, OI[iObj]).XYZ())
                                         + (voo[iObj] * parO[iObj]));
                        if (OI[iObj] > OI[inext])
                        {
                            parO[iObj] = 1.0 - parO[iObj];
                        }
                        piOT.Append(new Intf_SectionPoint(piO,
                                                          Intf_EDGE,
                                                          Math.Min(OI[iObj], OI[inext]),
                                                          Math.Max(OI[iObj], OI[inext]),
                                                          parO[iObj],
                                                          Intf_FACE,
                                                          Tri2,
                                                          0,
                                                          0.0,
                                                          Incidence));
                        nbpiOT++;
                        nbpiO++;
                    }
                }
            }

            // Intersection FACE EDGE
            for (iToo = 0; iToo < 3; iToo++)
            {
                jnext = Pourcent3[iToo + 1];
                if (edTT[iToo] == 1 && (dfOpT[iToo] * dfOpT[jnext]) < 0.0)
                {
                    lg = dfOpT[iToo] / (dfOpT[iToo] - dfOpT[jnext]);
                    if (lg > 0.0 && lg < 1.0)
                    {
                        parT[iToo] = lg;
                        piO = new Point3D((IntPatch_PolyhedronTool.Point(SeconPol, TI[iToo]).XYZ())
                                         + (vtt[iToo] * parT[iToo]));
                        if (TI[iToo] > TI[jnext])
                        {
                            parT[iToo] = 1.0 - parT[iToo];
                        }
                        piOT.Append(new Intf_SectionPoint(piO,
                                                          Intf_FACE,
                                                          Tri1,
                                                          0,
                                                          0.0,
                                                          Intf_EDGE,
                                                          Math.Min(TI[iToo], TI[jnext]),
                                                          Math.Max(TI[iToo], TI[jnext]),
                                                          parT[iToo],
                                                          Incidence));
                        nbpiOT++;
                        nbpiT++;
                    }
                }
            }

            // OCCT's NCollection_LocalArray: an int[] of at least four items, uninitialised beyond what is written.
            int[] id = new int[Math.Max(nbpiOT, 4)];

            int ideb = -1;
            int ifin = -2;

            if (nbpiOT > 1)
            {
                // Sort the <nbpiOT> sections points along the intersection between the
                // two triangles :

                Xyz dir = ONor ^ TNor;
                double[] d = new double[nbpiOT];
                int iPi, iPs;
                for (iPi = 0; iPi < nbpiOT; iPi++)
                {
                    d[iPi] = dir * piOT[iPi + 1].Pnt().XYZ();
                }

                int di;
                id[0] = 0;
                for (iPi = 1; iPi < nbpiOT; iPi++)
                {
                    id[iPi] = iPi;
                    for (iPs = iPi - 1; iPs >= 0; iPs--)
                    {
                        if (d[id[iPs]] > d[id[iPs + 1]])
                        {
                            di = id[iPs + 1];
                            id[iPs + 1] = id[iPs];
                            id[iPs] = di;
                        }
                        else
                        {
                            break;
                        }
                    }
                }
            }

            // Possibility of line of section :

            if (nbpiO == 2 && nbpiT == 2)
            {
                // In the case when an edge is in the plane of the other triangle
                // it is necessary to check if it has not been already processed
                // on a connected triangle :

                // Pour l objet :
                int pivo = -1;
                int pedg = -1;
                if (parO[0] == 0.0)
                {
                    pivo = 0;
                    if (parO[1] == 0.0)
                    {
                        pedg = 1;
                    }
                    else if (parO[2] == 0.0)
                    {
                        pedg = 2;
                    }
                }
                else if (parO[1] == 0.0)
                {
                    pivo = 1;
                    if (parO[2] == 0.0)
                    {
                        pedg = 2;
                    }
                }
                if (pivo >= 0 && pedg >= 0)
                {
                    IntPatch_PolyhedronTool.TriConnex(FirstPol, Tri1, OI[pivo], OI[pedg], out pivo, out pedg);
                    if (pivo > Tri1)
                    {
                        nbpiOT = 0;
                        ideb = -1; // On a deja trouve celle ci
                        ifin = -2;
                    }
                }

                // For the tool :
                pivo = -1;
                pedg = -1;
                if (parT[0] == 0.0)
                {
                    pivo = 0;
                    if (parT[1] == 0.0)
                    {
                        pedg = 1;
                    }
                    else if (parT[2] == 0.0)
                    {
                        pedg = 2;
                    }
                }
                else if (parT[1] == 0.0)
                {
                    pivo = 1;
                    if (parT[2] == 0.0)
                    {
                        pedg = 2;
                    }
                }
                if (pivo >= 0 && pedg >= 0)
                {
                    IntPatch_PolyhedronTool.TriConnex(SeconPol, Tri2, TI[pivo], TI[pedg], out pivo, out pedg);
                    if (pivo > Tri2)
                    {
                        nbpiOT = 0;
                        ideb = -1; // It has been already found
                        ifin = -2;
                    }
                }

                if (nbpiOT > 0)
                {
                    // If there is a covering up : insert the section  line in  the existent
                    // list or create a new section line :

                    if (piOT[id[0] + 1].TypeOnFirst() == Intf_FACE)
                    {
                        if (piOT[id[1] + 1].TypeOnFirst() == Intf_FACE)
                        {
                            ideb = -id[0] - 1; // No line of section possible
                            ifin = -id[1] - 1; //
                        }
                        else if (piOT[id[1] + 1].TypeOnSecond() != Intf_FACE)
                        {
                            ideb = id[1]; // No line of section possible
                            ifin = id[1]; // only a pointersec
                        }
                        else if (nbpiOT >= 3)
                        {
                            ideb = id[1]; // Retrieve 2 segments of section
                            ifin = id[2]; //
                        }
                        else
                        {
                            ideb = -999; // No line of section possible
                            ifin = -999;
                        }
                    }
                    else if (piOT[id[0] + 1].TypeOnSecond() == Intf_FACE)
                    {
                        if (piOT[id[1] + 1].TypeOnSecond() == Intf_FACE)
                        {
                            ideb = -id[0] - 1; // No line of section possible
                            ifin = -id[1] - 1; //
                        }
                        else if (piOT[id[1] + 1].TypeOnFirst() != Intf_FACE)
                        {
                            ideb = id[1]; // No line of section possible
                            ifin = id[1]; // only a pointersec
                        }
                        else if (nbpiOT >= 3)
                        {
                            ideb = id[1]; // Recouvrement des 2 segments de section
                            ifin = id[2]; //
                        }
                        else
                        {
                            ideb = -999; // No line of section possible
                            ifin = -999;
                        }
                    }

                    else
                    {               // Singularity on the first point there is only two or
                        ideb = id[0]; // three pointersec, so the first is a solution
                        ifin = id[1]; // and the second too.
                    }
                }

                // Insertion of the segment found in the existing section lines :

                if (ideb < 0)
                {
                    if (ifin < 0)
                    {
                        if (ideb != -999)
                        {
                            double d = piOT[-ideb].Pnt().Distance(piOT[-ifin].Pnt());
                            if (d < Tolerance)
                            {
                                Insert(piOT[-ideb], piOT[-ifin]);
                            }
                        }
                    }
                }
                else if (ideb >= 0)
                {
                    if (ideb != ifin)
                    {
                        Insert(piOT[ideb + 1], piOT[ifin + 1]);

                        //	else
                        //	 un pointersec : It is necessary to check if it has not been already found
                        //                       and if not insert it in the list.
                        //                       Attention! It is necessary to check
                        //                       for each new segment if a point is in the list
                        //                       and in this case remove it from the list.
                    }
                }
            }
        }
    }

    //=================================================================================================
    // Computes the zone of tangency between the facet Tri1 of FirstPol and the facet Tri2 of SeconPol.
    private bool TangentZoneValue(Intf_TangentZone TheTZ, IntPatch_Polyhedron FirstPol, int Tri1,
                                  IntPatch_Polyhedron SeconPol, int Tri2)
    {
        // Potential tangent Zone !
        // ------------------------

        bool finished = false;
        int nob, nou, nob2, nou2;
        double par;

        Span<Intf_PIType> tOP = stackalloc Intf_PIType[3];
        Span<Intf_PIType> tTP = stackalloc Intf_PIType[3];
        for (nou = 0; nou < 3; nou++)
        {
            tOP[nou] = Intf_EXTERNAL;
            tTP[nou] = Intf_EXTERNAL;
        }

        int nbpInt = 0;
        NCollection_Sequence<Intf_SectionPoint> Tpi = new();

        // Compute the positions of the points of <Tri1> in the triangle <Tri2>.
        for (nob = 0; nob <= 2; nob++)
        {
            nob2 = Pourcent3[nob + 1];
            for (nou = 0; nou <= 2; nou++)
            {
                nou2 = Pourcent3[nou + 1];
                if (dpOpT[nob, nou] <= Tolerance)
                {
                    Tpi.Append(new Intf_SectionPoint(IntPatch_PolyhedronTool.Point(FirstPol, OI[nob]),
                                                     Intf_VERTEX,
                                                     OI[nob],
                                                     0,
                                                     0.0,
                                                     Intf_VERTEX,
                                                     TI[nou],
                                                     0,
                                                     0.0,
                                                     1.0));
                    tOP[nob] = Intf_VERTEX;
                    tTP[nou] = Intf_VERTEX;
                    nbpInt++;
                    break;
                }
                else if (Math.Abs(dpOeT[nob, nou]) <= Tolerance)
                {
                    if (dpOpT[nob, nou] + dpOpT[nob, nou2] < vtt[nou].Modulus())
                    {
                        par = dpOpT[nob, nou] / (dpOpT[nob, nou] + dpOpT[nob, nou2]);
                        if (TI[nou] > TI[nou2])
                        {
                            par = 1.0 - par;
                        }
                        Tpi.Append(new Intf_SectionPoint(IntPatch_PolyhedronTool.Point(FirstPol, OI[nob]),
                                                         Intf_VERTEX,
                                                         OI[nob],
                                                         0,
                                                         0.0,
                                                         Intf_EDGE,
                                                         Math.Min(TI[nou], TI[nou2]),
                                                         Math.Max(TI[nou], TI[nou2]),
                                                         par,
                                                         1.0));
                        tOP[nob] = Intf_EDGE;
                        nbpInt++;
                        break;
                    }
                }
            }
            if (tOP[nob] == Intf_EXTERNAL)
            {
                if (Intf.Contain(IntPatch_PolyhedronTool.Point(SeconPol, TI[0]),
                                 IntPatch_PolyhedronTool.Point(SeconPol, TI[1]),
                                 IntPatch_PolyhedronTool.Point(SeconPol, TI[2]),
                                 IntPatch_PolyhedronTool.Point(FirstPol, OI[nob])))
                {
                    Tpi.Append(new Intf_SectionPoint(IntPatch_PolyhedronTool.Point(FirstPol, OI[nob]),
                                                     Intf_VERTEX,
                                                     OI[nob],
                                                     0,
                                                     0.0,
                                                     Intf_FACE,
                                                     Tri2,
                                                     0,
                                                     0.0,
                                                     1.0));
                    tOP[nob] = Intf_FACE;
                    nbpInt++;
                }
            }
        }

        // If the three points of <Tri1> are in <Tri2> the triangle Tri1 is
        // itself the tangent zone else compute the positions of the points
        // of <Tri2> in <Tri1>.
        if (nbpInt < 3)
        {
            for (nou = 0; nou <= 2; nou++)
            {
                nou2 = Pourcent3[nou + 1];
                if (tTP[nou] == Intf_EXTERNAL)
                {
                    for (nob = 0; nob <= 2; nob++)
                    {
                        nob2 = Pourcent3[nob + 1];
                        if (Math.Abs(deOpT[nob, nou]) <= Tolerance)
                        {
                            if (dpOpT[nob, nou] + dpOpT[nob2, nou] < voo[nob].Modulus())
                            {
                                par = dpOpT[nob, nou] / (dpOpT[nob, nou] + dpOpT[nob2, nou]);
                                if (OI[nob] > OI[nob2])
                                {
                                    par = 1.0 - par;
                                }
                                Tpi.Append(new Intf_SectionPoint(IntPatch_PolyhedronTool.Point(SeconPol, TI[nou]),
                                                                 Intf_EDGE,
                                                                 Math.Min(OI[nob], OI[nob2]),
                                                                 Math.Max(OI[nob], OI[nob2]),
                                                                 par,
                                                                 Intf_VERTEX,
                                                                 TI[nou],
                                                                 0,
                                                                 0.0,
                                                                 1.0));
                                tTP[nou] = Intf_EDGE;
                                nbpInt++;
                                break;
                            }
                        }
                    }
                    if (tTP[nou] == Intf_EXTERNAL)
                    {
                        if (Intf.Contain(IntPatch_PolyhedronTool.Point(FirstPol, OI[0]),
                                         IntPatch_PolyhedronTool.Point(FirstPol, OI[1]),
                                         IntPatch_PolyhedronTool.Point(FirstPol, OI[2]),
                                         IntPatch_PolyhedronTool.Point(SeconPol, TI[nou])))
                        {
                            Tpi.Append(new Intf_SectionPoint(IntPatch_PolyhedronTool.Point(SeconPol, TI[nou]),
                                                             Intf_FACE,
                                                             Tri1,
                                                             0,
                                                             0.0,
                                                             Intf_VERTEX,
                                                             TI[nou],
                                                             0,
                                                             0.0,
                                                             1.0));
                            tTP[nou] = Intf_FACE;
                            nbpInt++;
                        }
                    }
                }
            }
            if (tTP[0] != Intf_EXTERNAL && tTP[1] != Intf_EXTERNAL && tTP[2] != Intf_EXTERNAL)
            {
                finished = true;
            }
        }
        else
        {
            finished = true;
        }

        // Insertion of the points of intersection in the zone of tangency :
        for (nob = 1; nob <= nbpInt; nob++)
        {
            TheTZ.Append(Tpi[nob]);
        }

        if (!finished)
        {
            // If one of the triangles is not in the zone of tangency, it is necessary to find
            // the points of intersection edge/edge :

            // Last indexes are not used.
            // Arrays are increased to eliminate gcc warning.
            Span<double> parO = stackalloc double[10];
            Span<double> parT = stackalloc double[10];
            int nbNoInserted = 0;
            Span<int> piToInsert = stackalloc int[17]; // for GCC 4.9

            for (nob = 0; nob < 3; nob++)
            {
                // processing of the object segment P[nob], P[nob+1]
                nob2 = Pourcent3[nob + 1];

                for (nou = 0; nou < 3; nou++)
                {
                    // processing of the segment of the tool P[nou], P[nou+1]
                    nou2 = Pourcent3[nou + 1];

                    if (dpOeT[nob, nou] * dpOeT[nob2, nou] < 0.0 && deOpT[nob, nou] * deOpT[nob, nou2] < 0.0)
                    {
                        if (nbpInt >= 5)
                        {
                            break;
                        }
                        else
                        {
                            parO[nbpInt] = dpOeT[nob, nou] / (dpOeT[nob, nou] - dpOeT[nob2, nou]);
                            parT[nbpInt] = deOpT[nob, nou] / (deOpT[nob, nou] - deOpT[nob, nou2]);
                            Point3D lepi = IntPatch_PolyhedronTool.Point(SeconPol, TI[nou])
                                            .Translated(new Vector3D(vtt[nou] * parT[nbpInt]));
                            if (OI[nob] > OI[nob2])
                            {
                                parO[nbpInt] = 1.0 - parO[nbpInt];
                            }
                            if (TI[nou] > TI[nou2])
                            {
                                parT[nbpInt] = 1.0 - parT[nbpInt];
                            }
                            Tpi.Append(new Intf_SectionPoint(lepi,
                                                             Intf_EDGE,
                                                             Math.Min(OI[nob], OI[nob2]),
                                                             Math.Max(OI[nob], OI[nob2]),
                                                             parO[nbpInt],
                                                             Intf_EDGE,
                                                             Math.Min(TI[nou], TI[nou2]),
                                                             Math.Max(TI[nou], TI[nou2]),
                                                             parT[nbpInt],
                                                             Incidence));
                            nbpInt++;
                            if (!TheTZ.Insert(Tpi[nbpInt]))
                            {
                                piToInsert[nbNoInserted] = nbpInt;
                                nbNoInserted++;
                            }
                        }
                    }
                }
                if (nbpInt >= 5)
                {
                    break; // Number of pi passed in TZ !
                }
            }
            nob = nbNoInserted - 1;
            while (nob >= 0)
            {
                while (!TheTZ.Insert(Tpi[piToInsert[nob]]))
                {
                    nob--;
                    if (nob < 0)
                    {
                        break;
                    }
                }
                if (nob >= 0)
                {
                    nbNoInserted--;
                    while (nob < nbNoInserted)
                    {
                        piToInsert[nob] = piToInsert[nob + 1];
                        nob++;
                    }
                    nob = nbNoInserted - 1;
                }
            }
            // OCCT prints the points left in piToInsert here with Intf_SectionPoint::Dump, which prints nothing
            // in a release build: left out.
        }
        if (nbpInt < 3)
        {
            nbpInt = 0;
        }
        return nbpInt > 0;
    }

    //=================================================================================================
    // The values that characterise the couple of triangles OI and TI: dpOpT (point to point), dpOeT (point of the
    // object to edge of the tool) and deOpT (edge of the object to point of the tool), signed against the normal.
    private void CoupleCharacteristics(IntPatch_Polyhedron FirstPol, IntPatch_Polyhedron SeconPol)
    {
        int n1, n2;
        double lg;

        for (n1 = 0; n1 < 3; n1++)
        {
            n2 = Pourcent3[n1 + 1];
            voo[n1] = IntPatch_PolyhedronTool.Point(FirstPol, OI[n2]).XYZ()
                      - IntPatch_PolyhedronTool.Point(FirstPol, OI[n1]).XYZ();
            vtt[n1] = IntPatch_PolyhedronTool.Point(SeconPol, TI[n2]).XYZ()
                      - IntPatch_PolyhedronTool.Point(SeconPol, TI[n1]).XYZ();
        }

        Xyz vvec = (voo[0] ^ voo[1]) + (voo[1] ^ voo[2]) + (voo[2] ^ voo[0]);
        Xyz vnorT = (vtt[0] ^ vtt[1]) + (vtt[1] ^ vtt[2]) + (vtt[2] ^ vtt[0]);
        if (vnorT.Modulus() > vvec.Modulus())
        {
            vvec = vnorT;
        }

        for (n1 = 0; n1 < 3; n1++)
        {
            for (n2 = 0; n2 < 3; n2++)
            {
                Xyz vto = IntPatch_PolyhedronTool.Point(FirstPol, OI[n1]).XYZ()
                             - IntPatch_PolyhedronTool.Point(SeconPol, TI[n2]).XYZ();
                dpOpT[n1, n2] = vto.Modulus();

                lg = vtt[n2].Modulus();
                if (lg > 1e-16)
                { //-- RealEpsilon()
                    Xyz vv = vto ^ vtt[n2];
                    lg = (vvec * vv) > 0.0 ? lg : -lg;
                    dpOeT[n1, n2] = vv.Modulus() / lg;
                }
                else
                {
                    dpOeT[n1, n2] = dpOpT[n1, n2];
                }

                lg = voo[n1].Modulus();
                if (lg > 1e-16)
                { //-- RealEpsilon())
                    Xyz vv = vto ^ voo[n1];
                    lg = (vvec * vv) > 0.0 ? -lg : lg;
                    deOpT[n1, n2] = vv.Modulus() / lg;
                }
                else
                {
                    deOpT[n1, n2] = dpOpT[n1, n2];
                }
            }
        }
    }
}
