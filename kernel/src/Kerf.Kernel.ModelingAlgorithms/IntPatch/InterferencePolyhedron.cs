// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_InterferencePolyhedron.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_InterferencePolyhedron.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using Kerf.Engine.Intersection.Polygons;
using Kerf.Geometry;
using Occt;
using static Kerf.Engine.Intersection.Polygons.PIType;

namespace Kerf.Engine.Intersection.Patches;

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
public class InterferencePolyhedron : PolygonInterference
{
    private static readonly int[] Pourcent3 = [0, 1, 2, 0, 1, 2, 0, 1, 2];

    private readonly int[] _oi = new int[3];
    private readonly int[] _ti = new int[3];
    private readonly double[,] _dpOpT = new double[3, 3];
    private readonly double[,] _dpOeT = new double[3, 3];
    private readonly double[,] _deOpT = new double[3, 3];
    private readonly Xyz[] _voo = new Xyz[3];
    private readonly Xyz[] _vtt = new Xyz[3];
    private double _incidence;

    /// <summary>An empty interference of polyhedra.</summary>
    public InterferencePolyhedron()
        : base(false)
    {
        _incidence = 0;
    }

    /// <summary>Computes the interference of the two polyhedra.</summary>
    public InterferencePolyhedron(Polyhedron firstPol, Polyhedron seconPol)
        : base(false)
    {
        _incidence = 0;
        if (!PolyhedronTool.Bounding(firstPol).IsOut(PolyhedronTool.Bounding(seconPol)))
        {
            _tolerance = PolyhedronTool.DeflectionOverEstimation(firstPol)
                        + PolyhedronTool.DeflectionOverEstimation(seconPol);
            if (_tolerance == 0.0)
            {
                _tolerance = Epsilon(1000.0);
            }
            Interference(firstPol, seconPol);
        }
    }

    /// <summary>Computes the self-interference of the polyhedron.</summary>
    public InterferencePolyhedron(Polyhedron objet)
        : base(true)
    {
        _incidence = 0;
        _tolerance = PolyhedronTool.DeflectionOverEstimation(objet) * 2;
        if (_tolerance == 0.0)
        {
            _tolerance = Epsilon(1000.0);
        }
        Interference(objet, objet); //-- lbr le 5 juillet 96
    }

    /// <summary>Computes the interference of the two polyhedra.</summary>
    public void Perform(Polyhedron firstPol, Polyhedron seconPol)
    {
        SelfInterference(false);
        if (!PolyhedronTool.Bounding(firstPol).IsOut(PolyhedronTool.Bounding(seconPol)))
        {
            _tolerance = PolyhedronTool.DeflectionOverEstimation(firstPol)
                        + PolyhedronTool.DeflectionOverEstimation(seconPol);
            if (_tolerance == 0.0)
            {
                _tolerance = Epsilon(1000.0);
            }
            Interference(firstPol, seconPol);
        }
    }

    /// <summary>
    /// Sets the self-interference mode and its tolerance, and computes nothing: OCCT's self-interference
    /// <c>Interference(Obje)</c> is empty (see the remarks).
    /// </summary>
    public void Perform(Polyhedron objet)
    {
        SelfInterference(true);
        _tolerance = PolyhedronTool.DeflectionOverEstimation(objet) * 2;
        if (_tolerance == 0.0)
        {
            _tolerance = Epsilon(1000.0);
        }
        Interference(objet); // OCCT: an empty body (occt-issues.md, 5D)
    }

    private static void Interference(Polyhedron _)
    {
    }

    private void Interference(Polyhedron firstPol, Polyhedron secondPol)
    {
        // Build BVH sets for both polyhedra
        PolyhedronBvh set1 = new(firstPol);
        PolyhedronBvh set2 = new(secondPol);

        // Find candidate triangle pairs via BVH traversal
        BvhTraversal traversal = new();
        traversal.Perform(set1, set2, _selfIntf);

        // Process each candidate pair
        List<BvhTraversal.TrianglePair> pairs = traversal.Pairs();
        foreach (BvhTraversal.TrianglePair pair in pairs)
        {
            Intersect(pair.First, firstPol, pair.Second, secondPol);
        }
    }

    //=======================================================================
    // function : Intersect
    // purpose  : Intersection of two triangles issue from two Polyhedron.
    //=======================================================================
    private void Intersect(int tri1, Polyhedron firstPol, int tri2, Polyhedron seconPol)
    {
        PolyhedronTool.Triangle(firstPol, tri1, out _oi[0], out _oi[1], out _oi[2]);
        PolyhedronTool.Triangle(seconPol, tri2, out _ti[0], out _ti[1], out _ti[2]);

        // If there is an intersection of a polyhedron with itself, the
        // intersections are excluded
        // from a triangle with connected triangles :

        if (_selfIntf)
        {
            if (_oi[0] == _ti[0] || _oi[0] == _ti[1] || _oi[0] == _ti[2] || _oi[1] == _ti[0] || _oi[1] == _ti[1]
                || _oi[1] == _ti[2] || _oi[2] == _ti[0] || _oi[2] == _ti[1] || _oi[2] == _ti[2])
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
        global::Kerf.Engine.Intersection.Polygons.Intf.PlaneEquation(PolyhedronTool.Point(firstPol, _oi[0]),
                           PolyhedronTool.Point(firstPol, _oi[1]),
                           PolyhedronTool.Point(firstPol, _oi[2]),
                           out Xyz oNor, // Normal vector.
                           out double odp); // Polar Distance.

        // Equation of the triangle plane of the tool
        global::Kerf.Engine.Intersection.Polygons.Intf.PlaneEquation(PolyhedronTool.Point(seconPol, _ti[0]),
                           PolyhedronTool.Point(seconPol, _ti[1]),
                           PolyhedronTool.Point(seconPol, _ti[2]),
                           out Xyz tNor, // Normal vector.
                           out double tdp); // Polar distance.

        // Scalar product of two normalized vectors -> cosinus of the angle
        _incidence = Math.Abs(tNor * oNor);

        // Distance of the plane of the triangle from the object by three points of SeconPol
        Span<double> dfOpT = stackalloc double[3];
        dfOpT[0] = oNor * (PolyhedronTool.Point(seconPol, _ti[0]).XYZ()) - odp;
        dfOpT[1] = oNor * (PolyhedronTool.Point(seconPol, _ti[1]).XYZ()) - odp;
        dfOpT[2] = oNor * (PolyhedronTool.Point(seconPol, _ti[2]).XYZ()) - odp;

        // Distance of the plane of the triangle from the tool by three points of FirstPol
        Span<double> dpOfT = stackalloc double[3];
        dpOfT[0] = tNor * (PolyhedronTool.Point(firstPol, _oi[0]).XYZ()) - tdp;
        dpOfT[1] = tNor * (PolyhedronTool.Point(firstPol, _oi[1]).XYZ()) - tdp;
        dpOfT[2] = tNor * (PolyhedronTool.Point(firstPol, _oi[2]).XYZ()) - tdp;

        // Values defining the couple of triangle dpOpT, dpOeT, deOpT
        CoupleCharacteristics(firstPol, seconPol);

        // If three  points  of the triangle of <SeconPol> are in the plane of the
        // triangle of <Obje> within <Tolerance> the eventual tangency zone is found.

        TangentZone theTZ = new();
        if ((Math.Abs(dfOpT[0]) <= _tolerance && Math.Abs(dfOpT[1]) <= _tolerance && Math.Abs(dfOpT[2]) <= _tolerance)
            && (Math.Abs(dpOfT[0]) <= _tolerance && Math.Abs(dpOfT[1]) <= _tolerance
                && Math.Abs(dpOfT[2]) <= _tolerance)
            && (Math.Abs(dfOpT[0] + dfOpT[1] + dfOpT[2])
                != Math.Abs(dfOpT[0]) + Math.Abs(dfOpT[1]) + Math.Abs(dfOpT[2]))
            && (Math.Abs(dpOfT[0] + dpOfT[1] + dpOfT[2])
                != Math.Abs(dpOfT[0]) + Math.Abs(dpOfT[1]) + Math.Abs(dpOfT[2])))
        {
            if (TangentZoneValue(theTZ, firstPol, tri1, seconPol, tri2))
            {
                if (!Insert(theTZ))
                {
                    _tZones.Append(theTZ);
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
            NCollection_Sequence<SectionPoint> piOT = new();
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
                    if (_dpOpT[iObj, iToo] <= floatGap)
                    {
                        piOT.Append(new SectionPoint(PolyhedronTool.Point(firstPol, _oi[iObj]),
                                                          PIType.Vertex,
                                                          _oi[iObj],
                                                          0,
                                                          0.0,
                                                          PIType.Vertex,
                                                          _ti[iToo],
                                                          0,
                                                          0.0,
                                                          _incidence));
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
                            if (_dpOeT[iObj, iToo] <= floatGap && _dpOeT[iObj, iToo] >= -floatGap)
                            {
                                if ((_dpOpT[iObj, iToo] + _dpOpT[iObj, inext]) < _vtt[iToo].Modulus())
                                {
                                    parT[iToo] = _dpOpT[iObj, iToo] / (_dpOpT[iObj, iToo] + _dpOpT[iObj, inext]);
                                    if (_ti[iToo] > _ti[inext])
                                    {
                                        parT[iToo] = 1.0 - parT[iToo];
                                    }
                                    piOT.Append(new SectionPoint(PolyhedronTool.Point(firstPol, _oi[iObj]),
                                                                      PIType.Vertex,
                                                                      _oi[iObj],
                                                                      0,
                                                                      0.0,
                                                                      PIType.Edge,
                                                                      Math.Min(_ti[iToo], _ti[inext]),
                                                                      Math.Max(_ti[iToo], _ti[inext]),
                                                                      parT[iToo],
                                                                      _incidence));
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
                            if (Math.Abs(_deOpT[iObj, iToo]) <= floatGap)
                            {
                                if ((_dpOpT[iObj, iToo] + _dpOpT[inext, iToo]) < _voo[iObj].Modulus())
                                {
                                    parO[iObj] = _dpOpT[iObj, iToo] / (_dpOpT[iObj, iToo] + _dpOpT[inext, iToo]);
                                    if (_oi[iObj] > _oi[inext])
                                    {
                                        parO[iObj] = 1.0 - parO[iObj];
                                    }
                                    piOT.Append(new SectionPoint(PolyhedronTool.Point(seconPol, _ti[iToo]),
                                                                      PIType.Edge,
                                                                      Math.Min(_oi[iObj], _oi[inext]),
                                                                      Math.Max(_oi[iObj], _oi[inext]),
                                                                      parO[iObj],
                                                                      PIType.Vertex,
                                                                      _ti[iToo],
                                                                      0,
                                                                      0.0,
                                                                      _incidence));
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
                        piOT.Append(new SectionPoint(PolyhedronTool.Point(seconPol, _ti[iToo]),
                                                          PIType.Face,
                                                          tri1,
                                                          0,
                                                          0.0,
                                                          PIType.Vertex,
                                                          _ti[iToo],
                                                          0,
                                                          0.0,
                                                          _incidence));
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
                        piOT.Append(new SectionPoint(PolyhedronTool.Point(firstPol, _oi[iObj]),
                                                          PIType.Vertex,
                                                          _oi[iObj],
                                                          0,
                                                          0.0,
                                                          PIType.Face,
                                                          tri2,
                                                          0,
                                                          0.0,
                                                          _incidence));
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
                                    bool pb = false;
                                    if (_oi[iObj] > _oi[inext])
                                    {
                                        double div = (_dpOeT[inext, iToo] - _dpOeT[iObj, iToo]);
                                        if (div > floatGap || div < -floatGap)
                                        {
                                            parO[iObj] = _dpOeT[inext, iToo] / div;
                                            piO = new Point3D((PolyhedronTool.Point(firstPol, _oi[inext]).XYZ())
                                                             + (_voo[iObj].Reversed() * parO[iObj]));
                                        }
                                        else
                                        {
                                            pb = true;
                                        }
                                    }
                                    else
                                    {
                                        double div = _dpOeT[iObj, iToo] - _dpOeT[inext, iToo];
                                        if (div > floatGap || div < -floatGap)
                                        {
                                            parO[iObj] = _dpOeT[iObj, iToo] / (_dpOeT[iObj, iToo] - _dpOeT[inext, iToo]);
                                            piO = new Point3D((PolyhedronTool.Point(firstPol, _oi[iObj]).XYZ())
                                                             + (_voo[iObj] * parO[iObj]));
                                        }
                                        else
                                        {
                                            pb = true;
                                        }
                                    }
                                    if (_ti[iToo] > _ti[jnext])
                                    {
                                        double div = (_deOpT[iObj, jnext] - _deOpT[iObj, iToo]);
                                        if (div > floatGap || div < -floatGap)
                                        {
                                            parT[iToo] = _deOpT[iObj, jnext] / div;
                                            piT = (PolyhedronTool.Point(seconPol, _ti[jnext]).XYZ())
                                                  + (_vtt[iToo].Reversed() * parT[iToo]);
                                        }
                                        else
                                        {
                                            pb = true;
                                        }
                                    }
                                    else
                                    {
                                        double div = (_deOpT[iObj, iToo] - _deOpT[iObj, jnext]);
                                        if (div > floatGap || div < -floatGap)
                                        {
                                            parT[iToo] = _deOpT[iObj, iToo] / div;
                                            piT = (PolyhedronTool.Point(seconPol, _ti[iToo]).XYZ())
                                                  + (_vtt[iToo] * parT[iToo]);
                                        }
                                        else
                                        {
                                            pb = true;
                                        }
                                    }
                                    if (!pb)
                                    {
                                        piT -= piO.XYZ();
                                        lg = piT.Modulus();
                                        if (lg <= floatGap)
                                        {
                                            piOT.Append(new SectionPoint(piO,
                                                                              PIType.Edge,
                                                                              Math.Min(_oi[iObj], _oi[inext]),
                                                                              Math.Max(_oi[iObj], _oi[inext]),
                                                                              parO[iObj],
                                                                              PIType.Edge,
                                                                              Math.Min(_ti[iToo], _ti[jnext]),
                                                                              Math.Max(_ti[iToo], _ti[jnext]),
                                                                              parT[iToo],
                                                                              _incidence));
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
                        piO = new Point3D((PolyhedronTool.Point(firstPol, _oi[iObj]).XYZ())
                                         + (_voo[iObj] * parO[iObj]));
                        if (_oi[iObj] > _oi[inext])
                        {
                            parO[iObj] = 1.0 - parO[iObj];
                        }
                        piOT.Append(new SectionPoint(piO,
                                                          PIType.Edge,
                                                          Math.Min(_oi[iObj], _oi[inext]),
                                                          Math.Max(_oi[iObj], _oi[inext]),
                                                          parO[iObj],
                                                          PIType.Face,
                                                          tri2,
                                                          0,
                                                          0.0,
                                                          _incidence));
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
                        piO = new Point3D((PolyhedronTool.Point(seconPol, _ti[iToo]).XYZ())
                                         + (_vtt[iToo] * parT[iToo]));
                        if (_ti[iToo] > _ti[jnext])
                        {
                            parT[iToo] = 1.0 - parT[iToo];
                        }
                        piOT.Append(new SectionPoint(piO,
                                                          PIType.Face,
                                                          tri1,
                                                          0,
                                                          0.0,
                                                          PIType.Edge,
                                                          Math.Min(_ti[iToo], _ti[jnext]),
                                                          Math.Max(_ti[iToo], _ti[jnext]),
                                                          parT[iToo],
                                                          _incidence));
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

                Xyz dir = oNor ^ tNor;
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
                    PolyhedronTool.TriConnex(firstPol, tri1, _oi[pivo], _oi[pedg], out pivo, out pedg);
                    if (pivo > tri1)
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
                    PolyhedronTool.TriConnex(seconPol, tri2, _ti[pivo], _ti[pedg], out pivo, out pedg);
                    if (pivo > tri2)
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

                    if (piOT[id[0] + 1].TypeOnFirst() == PIType.Face)
                    {
                        if (piOT[id[1] + 1].TypeOnFirst() == PIType.Face)
                        {
                            ideb = -id[0] - 1; // No line of section possible
                            ifin = -id[1] - 1; //
                        }
                        else if (piOT[id[1] + 1].TypeOnSecond() != PIType.Face)
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
                    else if (piOT[id[0] + 1].TypeOnSecond() == PIType.Face)
                    {
                        if (piOT[id[1] + 1].TypeOnSecond() == PIType.Face)
                        {
                            ideb = -id[0] - 1; // No line of section possible
                            ifin = -id[1] - 1; //
                        }
                        else if (piOT[id[1] + 1].TypeOnFirst() != PIType.Face)
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
                            if (d < _tolerance)
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
    private bool TangentZoneValue(TangentZone theTZ, Polyhedron firstPol, int tri1,
                                  Polyhedron seconPol, int tri2)
    {
        // Potential tangent Zone !
        // ------------------------

        bool finished = false;
        int nob, nou, nob2, nou2;
        double par;

        Span<PIType> tOP = stackalloc PIType[3];
        Span<PIType> tTP = stackalloc PIType[3];
        for (nou = 0; nou < 3; nou++)
        {
            tOP[nou] = PIType.External;
            tTP[nou] = PIType.External;
        }

        int nbpInt = 0;
        NCollection_Sequence<SectionPoint> tpi = new();

        // Compute the positions of the points of <Tri1> in the triangle <Tri2>.
        for (nob = 0; nob <= 2; nob++)
        {
            nob2 = Pourcent3[nob + 1];
            for (nou = 0; nou <= 2; nou++)
            {
                nou2 = Pourcent3[nou + 1];
                if (_dpOpT[nob, nou] <= _tolerance)
                {
                    tpi.Append(new SectionPoint(PolyhedronTool.Point(firstPol, _oi[nob]),
                                                     PIType.Vertex,
                                                     _oi[nob],
                                                     0,
                                                     0.0,
                                                     PIType.Vertex,
                                                     _ti[nou],
                                                     0,
                                                     0.0,
                                                     1.0));
                    tOP[nob] = PIType.Vertex;
                    tTP[nou] = PIType.Vertex;
                    nbpInt++;
                    break;
                }
                else if (Math.Abs(_dpOeT[nob, nou]) <= _tolerance)
                {
                    if (_dpOpT[nob, nou] + _dpOpT[nob, nou2] < _vtt[nou].Modulus())
                    {
                        par = _dpOpT[nob, nou] / (_dpOpT[nob, nou] + _dpOpT[nob, nou2]);
                        if (_ti[nou] > _ti[nou2])
                        {
                            par = 1.0 - par;
                        }
                        tpi.Append(new SectionPoint(PolyhedronTool.Point(firstPol, _oi[nob]),
                                                         PIType.Vertex,
                                                         _oi[nob],
                                                         0,
                                                         0.0,
                                                         PIType.Edge,
                                                         Math.Min(_ti[nou], _ti[nou2]),
                                                         Math.Max(_ti[nou], _ti[nou2]),
                                                         par,
                                                         1.0));
                        tOP[nob] = PIType.Edge;
                        nbpInt++;
                        break;
                    }
                }
            }
            if (tOP[nob] == PIType.External)
            {
                if (global::Kerf.Engine.Intersection.Polygons.Intf.Contain(PolyhedronTool.Point(seconPol, _ti[0]),
                                 PolyhedronTool.Point(seconPol, _ti[1]),
                                 PolyhedronTool.Point(seconPol, _ti[2]),
                                 PolyhedronTool.Point(firstPol, _oi[nob])))
                {
                    tpi.Append(new SectionPoint(PolyhedronTool.Point(firstPol, _oi[nob]),
                                                     PIType.Vertex,
                                                     _oi[nob],
                                                     0,
                                                     0.0,
                                                     PIType.Face,
                                                     tri2,
                                                     0,
                                                     0.0,
                                                     1.0));
                    tOP[nob] = PIType.Face;
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
                if (tTP[nou] == PIType.External)
                {
                    for (nob = 0; nob <= 2; nob++)
                    {
                        nob2 = Pourcent3[nob + 1];
                        if (Math.Abs(_deOpT[nob, nou]) <= _tolerance)
                        {
                            if (_dpOpT[nob, nou] + _dpOpT[nob2, nou] < _voo[nob].Modulus())
                            {
                                par = _dpOpT[nob, nou] / (_dpOpT[nob, nou] + _dpOpT[nob2, nou]);
                                if (_oi[nob] > _oi[nob2])
                                {
                                    par = 1.0 - par;
                                }
                                tpi.Append(new SectionPoint(PolyhedronTool.Point(seconPol, _ti[nou]),
                                                                 PIType.Edge,
                                                                 Math.Min(_oi[nob], _oi[nob2]),
                                                                 Math.Max(_oi[nob], _oi[nob2]),
                                                                 par,
                                                                 PIType.Vertex,
                                                                 _ti[nou],
                                                                 0,
                                                                 0.0,
                                                                 1.0));
                                tTP[nou] = PIType.Edge;
                                nbpInt++;
                                break;
                            }
                        }
                    }
                    if (tTP[nou] == PIType.External)
                    {
                        if (global::Kerf.Engine.Intersection.Polygons.Intf.Contain(PolyhedronTool.Point(firstPol, _oi[0]),
                                         PolyhedronTool.Point(firstPol, _oi[1]),
                                         PolyhedronTool.Point(firstPol, _oi[2]),
                                         PolyhedronTool.Point(seconPol, _ti[nou])))
                        {
                            tpi.Append(new SectionPoint(PolyhedronTool.Point(seconPol, _ti[nou]),
                                                             PIType.Face,
                                                             tri1,
                                                             0,
                                                             0.0,
                                                             PIType.Vertex,
                                                             _ti[nou],
                                                             0,
                                                             0.0,
                                                             1.0));
                            tTP[nou] = PIType.Face;
                            nbpInt++;
                        }
                    }
                }
            }
            if (tTP[0] != PIType.External && tTP[1] != PIType.External && tTP[2] != PIType.External)
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
            theTZ.Append(tpi[nob]);
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

                    if (_dpOeT[nob, nou] * _dpOeT[nob2, nou] < 0.0 && _deOpT[nob, nou] * _deOpT[nob, nou2] < 0.0)
                    {
                        if (nbpInt >= 5)
                        {
                            break;
                        }
                        else
                        {
                            parO[nbpInt] = _dpOeT[nob, nou] / (_dpOeT[nob, nou] - _dpOeT[nob2, nou]);
                            parT[nbpInt] = _deOpT[nob, nou] / (_deOpT[nob, nou] - _deOpT[nob, nou2]);
                            Point3D lepi = PolyhedronTool.Point(seconPol, _ti[nou])
                                            .Translated(new Vector3D(_vtt[nou] * parT[nbpInt]));
                            if (_oi[nob] > _oi[nob2])
                            {
                                parO[nbpInt] = 1.0 - parO[nbpInt];
                            }
                            if (_ti[nou] > _ti[nou2])
                            {
                                parT[nbpInt] = 1.0 - parT[nbpInt];
                            }
                            tpi.Append(new SectionPoint(lepi,
                                                             PIType.Edge,
                                                             Math.Min(_oi[nob], _oi[nob2]),
                                                             Math.Max(_oi[nob], _oi[nob2]),
                                                             parO[nbpInt],
                                                             PIType.Edge,
                                                             Math.Min(_ti[nou], _ti[nou2]),
                                                             Math.Max(_ti[nou], _ti[nou2]),
                                                             parT[nbpInt],
                                                             _incidence));
                            nbpInt++;
                            if (!theTZ.Insert(tpi[nbpInt]))
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
                while (!theTZ.Insert(tpi[piToInsert[nob]]))
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
    private void CoupleCharacteristics(Polyhedron firstPol, Polyhedron seconPol)
    {
        int n1, n2;
        double lg;

        for (n1 = 0; n1 < 3; n1++)
        {
            n2 = Pourcent3[n1 + 1];
            _voo[n1] = PolyhedronTool.Point(firstPol, _oi[n2]).XYZ()
                      - PolyhedronTool.Point(firstPol, _oi[n1]).XYZ();
            _vtt[n1] = PolyhedronTool.Point(seconPol, _ti[n2]).XYZ()
                      - PolyhedronTool.Point(seconPol, _ti[n1]).XYZ();
        }

        Xyz vvec = (_voo[0] ^ _voo[1]) + (_voo[1] ^ _voo[2]) + (_voo[2] ^ _voo[0]);
        Xyz vnorT = (_vtt[0] ^ _vtt[1]) + (_vtt[1] ^ _vtt[2]) + (_vtt[2] ^ _vtt[0]);
        if (vnorT.Modulus() > vvec.Modulus())
        {
            vvec = vnorT;
        }

        for (n1 = 0; n1 < 3; n1++)
        {
            for (n2 = 0; n2 < 3; n2++)
            {
                Xyz vto = PolyhedronTool.Point(firstPol, _oi[n1]).XYZ()
                             - PolyhedronTool.Point(seconPol, _ti[n2]).XYZ();
                _dpOpT[n1, n2] = vto.Modulus();

                lg = _vtt[n2].Modulus();
                if (lg > 1e-16)
                { //-- RealEpsilon()
                    Xyz vv = vto ^ _vtt[n2];
                    lg = (vvec * vv) > 0.0 ? lg : -lg;
                    _dpOeT[n1, n2] = vv.Modulus() / lg;
                }
                else
                {
                    _dpOeT[n1, n2] = _dpOpT[n1, n2];
                }

                lg = _voo[n1].Modulus();
                if (lg > 1e-16)
                { //-- RealEpsilon())
                    Xyz vv = vto ^ _voo[n1];
                    lg = (vvec * vv) > 0.0 ? -lg : lg;
                    _deOpT[n1, n2] = vv.Modulus() / lg;
                }
                else
                {
                    _deOpT[n1, n2] = _dpOpT[n1, n2];
                }
            }
        }
    }
}
