// Port of: src/ModelingData/TKBRep/BRepTools/BRepTools_TrsfModification.hxx
// Port of: src/ModelingData/TKBRep/BRepTools/BRepTools_TrsfModification.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GeomAbs_SurfaceType;
using static Occt.gp_TrsfForm;
using static Occt.TopAbs_Orientation;

namespace Occt;

/// <summary>
/// The modification that applies a transformation (gp_Trsf) to the geometry of a shape: surfaces, curves, points,
/// pcurves (reparametrised where the transformation changes the parametrisation) and, if <see cref="IsCopyMesh"/>,
/// triangulations and polygons.
/// </summary>
public class BRepTools_TrsfModification : BRepTools_Modification
{
    private gp_Trsf myTrsf;
    private bool myCopyMesh;

    /// <summary>The modification applying <paramref name="T"/>; meshes are not copied.</summary>
    public BRepTools_TrsfModification(in gp_Trsf T)
    {
        myTrsf = T;
        myCopyMesh = false;
    }

    /// <summary>The transformation, for reading and writing (a working object).</summary>
    public ref gp_Trsf Trsf() => ref myTrsf;

    /// <summary>Whether triangulations and polygons are transformed too, for reading and writing.</summary>
    public ref bool IsCopyMesh() => ref myCopyMesh;

    /// <summary>The face's surface transformed (in the frame of its location); false if the face has no surface.</summary>
    public override bool NewSurface(in TopoDS_Face F,
                                    out Geom_Surface? S,
                                    out TopLoc_Location L,
                                    out double Tol,
                                    out bool RevWires,
                                    out bool RevFace)
    {
        Tol = 0.0;
        RevWires = RevFace = false;
        S = BRep_Tool.Surface(F, out L);
        if (S is null)
        {
            // processing cases when there is no geometry
            return false;
        }

        Tol = BRep_Tool.Tolerance(F);
        Tol *= Math.Abs(myTrsf.ScaleFactor());
        RevWires = false;
        RevFace = myTrsf.IsNegative();

        gp_Trsf LT = L.Transformation();
        LT.Invert();
        LT.Multiply(myTrsf);
        LT.Multiply(L.Transformation());

        S = (Geom_Surface)S.Transformed(LT);

        return true;
    }

    /// <summary>The face's triangulation transformed, if meshes are copied.</summary>
    public override bool NewTriangulation(in TopoDS_Face theFace, out Poly_Triangulation? theTriangulation)
    {
        theTriangulation = null;
        if (!myCopyMesh)
        {
            return false;
        }

        theTriangulation = BRep_Tool.Triangulation(theFace, out TopLoc_Location aLoc);

        if (theTriangulation is null)
        {
            return false;
        }

        gp_Trsf aTrsf = myTrsf;
        if (!aLoc.IsIdentity())
        {
            aTrsf = aLoc.Transformation().Inverted() * aTrsf * aLoc.Transformation();
        }

        theTriangulation = theTriangulation.Copy();
        theTriangulation.SetCachedMinMax(new Bnd_Box()); // clear bounding box
        theTriangulation.Deflection(theTriangulation.Deflection() * Math.Abs(myTrsf.ScaleFactor()));
        // apply transformation to 3D nodes
        for (int anInd = 1; anInd <= theTriangulation.NbNodes(); ++anInd)
        {
            gp_Pnt aP = theTriangulation.Node(anInd);
            aP.Transform(aTrsf);
            theTriangulation.SetNode(anInd, aP);
        }
        // modify 2D nodes
        Geom_Surface? aSurf = BRep_Tool.Surface(theFace, out aLoc);
        if (theTriangulation.HasUVNodes() && aSurf is not null)
        {
            for (int anInd = 1; anInd <= theTriangulation.NbNodes(); ++anInd)
            {
                gp_Pnt2d aP2d = theTriangulation.UVNode(anInd);
                double aU = aP2d.X(), aV = aP2d.Y();
                aSurf.TransformParameters(ref aU, ref aV, myTrsf);
                aP2d.SetCoord(aU, aV);
                theTriangulation.SetUVNode(anInd, aP2d);
            }
        }
        // modify triangles orientation in case of mirror transformation
        if (myTrsf.ScaleFactor() < 0.0)
        {
            for (int anInd = 1; anInd <= theTriangulation.NbTriangles(); ++anInd)
            {
                Poly_Triangle aTria = theTriangulation.Triangle(anInd);
                aTria.Get(out int aN1, out int aN2, out int aN3);
                aTria.Set(aN1, aN3, aN2);
                theTriangulation.SetTriangle(anInd, aTria);
            }
        }
        // modify normals
        if (theTriangulation.HasNormals())
        {
            for (int anInd = 1; anInd <= theTriangulation.NbNodes(); ++anInd)
            {
                gp_Dir aNormal = theTriangulation.Normal(anInd);
                aNormal.Transform(aTrsf);
                theTriangulation.SetNormal(anInd, aNormal);
            }
        }

        return true;
    }

    /// <summary>The edge's 3D polygon transformed (its parameters too), if meshes are copied.</summary>
    public override bool NewPolygon(in TopoDS_Edge theE, out Poly_Polygon3D? theP)
    {
        theP = null;
        if (!myCopyMesh)
        {
            return false;
        }

        theP = BRep_Tool.Polygon3D(theE, out TopLoc_Location aLoc);
        if (theP is null)
        {
            return false;
        }

        gp_Trsf aTrsf = myTrsf;
        if (!aLoc.IsIdentity())
        {
            aTrsf = aLoc.Transformation().Inverted() * aTrsf * aLoc.Transformation();
        }

        theP = theP.Copy();
        theP.Deflection(theP.Deflection() * Math.Abs(myTrsf.ScaleFactor()));
        NCollection_Array1<gp_Pnt> aNodesArray = theP.ChangeNodes();
        for (int anId = aNodesArray.Lower(); anId <= aNodesArray.Upper(); ++anId)
        {
            // Applying the transformation to each node of polygon
            aNodesArray[anId].Transform(aTrsf);
        }
        // transform the parametrization
        if (theP.HasParameters())
        {
            Geom_Curve? aCurve = BRep_Tool.Curve(theE, out _, out _, out _);
            if (aCurve is not null)
            {
                double aReparametrization = aCurve.ParametricTransformation(aTrsf);
                if (Math.Abs(aReparametrization - 1.0) > Precision.PConfusion())
                {
                    NCollection_Array1<double> aParams = theP.ChangeParameters();
                    for (int anInd = aParams.Lower(); anInd <= aParams.Upper(); ++anInd)
                    {
                        aParams[anInd] *= aReparametrization;
                    }
                }
            }
        }
        return true;
    }

    /// <summary>The edge's polygon on the face's triangulation, its parameters mapped to the new pcurve, if meshes are copied.</summary>
    public override bool NewPolygonOnTriangulation(in TopoDS_Edge theE, in TopoDS_Face theF, out Poly_PolygonOnTriangulation? theP)
    {
        theP = null;
        if (!myCopyMesh)
        {
            return false;
        }

        Poly_Triangulation? aT = BRep_Tool.Triangulation(theF, out TopLoc_Location aLoc);
        if (aT is null)
        {
            theP = null;
            return false;
        }

        theP = BRep_Tool.PolygonOnTriangulation(theE, aT, aLoc);
        if (theP is null)
        {
            return false;
        }
        theP = theP.Copy();
        theP.Deflection(theP.Deflection() * Math.Abs(myTrsf.ScaleFactor()));

        // transform the parametrization
        Geom_Surface? aSurf = BRep_Tool.Surface(theF, out aLoc);
        Geom2d_Curve? aC2d = BRep_Tool.CurveOnSurface(theE, theF, out _, out _);
        if (aSurf is not null && aC2d is not null
            && Math.Abs(Math.Abs(myTrsf.ScaleFactor()) - 1.0) > TopLoc_Location.ScalePrec())
        {
            gp_GTrsf2d aGTrsf = aSurf.ParametricTransformation(myTrsf);
            if (aGTrsf.Form() != gp_Identity)
            {
                Geom2d_Curve? aNewC2d = GeomLib.GTransform(aC2d, aGTrsf);
                for (int anInd = 1; anInd <= theP.NbNodes(); ++anInd)
                {
                    double aParam = theP.Parameter(anInd);
                    gp_Pnt2d aP2d = aC2d.Value(aParam);
                    gp_XY aXY = aP2d.XY();
                    aGTrsf.Transforms(ref aXY);
                    aP2d.SetXY(aXY);
                    GeomLib_Tool.Parameter(aNewC2d, aP2d, theP.Deflection(), out aParam);
                    theP.SetParameter(anInd, aParam);
                }
            }
        }

        return true;
    }

    /// <summary>The edge's 3D curve transformed (in the frame of its location); always true.</summary>
    public override bool NewCurve(in TopoDS_Edge E, out Geom_Curve? C, out TopLoc_Location L, out double Tol)
    {
        C = BRep_Tool.Curve(E, out L, out _, out _);

        Tol = BRep_Tool.Tolerance(E);
        Tol *= Math.Abs(myTrsf.ScaleFactor());

        gp_Trsf LT = L.Transformation();
        LT.Invert();
        LT.Multiply(myTrsf);
        LT.Multiply(L.Transformation());

        if (C is not null)
        {
            C = (Geom_Curve)C.Transformed(LT);
        }

        return true;
    }

    /// <summary>The vertex's point transformed; always true.</summary>
    public override bool NewPoint(in TopoDS_Vertex V, out gp_Pnt P, out double Tol)
    {
        P = BRep_Tool.Pnt(V);
        Tol = BRep_Tool.Tolerance(V);
        Tol *= Math.Abs(myTrsf.ScaleFactor());
        P.Transform(myTrsf);

        return true;
    }

    /// <summary>
    /// The edge's pcurve on the face, transformed in the parametric space when the transformation scales, with the
    /// range of the new vertex parameters; false on a plane or a face without surface.
    /// </summary>
    public override bool NewCurve2d(in TopoDS_Edge E,
                                    in TopoDS_Face F,
                                    in TopoDS_Edge NewE,
                                    in TopoDS_Face NewF,
                                    out Geom2d_Curve? C,
                                    out double Tol)
    {
        C = null;
        Tol = BRep_Tool.Tolerance(E);
        double scale = myTrsf.ScaleFactor();
        Tol *= Math.Abs(scale);
        Geom_Surface? S = BRep_Tool.Surface(F, out _);

        if (S is null)
        {
            // processing the case when the surface (geometry) is deleted
            return false;
        }
        GeomAdaptor_Surface GAsurf = new GeomAdaptor_Surface(S);
        if (GAsurf.GetType() == GeomAbs_Plane)
        {
            return false;
        }

        Geom2d_Curve? NewC = BRep_Tool.CurveOnSurface(E, F, out double f, out double l);
        if (NewC is null)
        {
            return false;
        }

        double newf, newl;

        if (NewC.GetType() == typeof(Geom2d_TrimmedCurve))
        {
            Geom2d_TrimmedCurve TC = (Geom2d_TrimmedCurve)NewC;
            NewC = TC.BasisCurve();
        }

        double fc = NewC.FirstParameter(), lc = NewC.LastParameter();

        if (!NewC.IsPeriodic())
        {
            if (fc - f > Precision.PConfusion())
            {
                f = fc;
            }
            if (l - lc > Precision.PConfusion())
            {
                l = lc;
            }
            if (Math.Abs(l - f) < Precision.PConfusion())
            {
                if (Math.Abs(f - fc) < Precision.PConfusion() && !Precision.IsInfinite(lc))
                {
                    l = lc;
                }
                else if (!Precision.IsInfinite(fc))
                {
                    f = fc;
                }
            }
        }

        newf = f;
        newl = l;
        if (Math.Abs(scale) != 1.0)
        {
            NewC = new Geom2d_TrimmedCurve(NewC, f, l);
            gp_GTrsf2d gtrsf = S.ParametricTransformation(myTrsf);

            if (gtrsf.Form() != gp_Identity)
            {
                NewC = GeomLib.GTransform(NewC, gtrsf);
                if (NewC is null)
                {
                    throw new Standard_DomainError("TrsfModification:Error in NewCurve2d");
                }
                newf = NewC.FirstParameter();
                newl = NewC.LastParameter();
            }
        }
        // sometimes need to readjust the 3D / 2D ranges
        TopExp.Vertices(E, out TopoDS_Vertex V1, out TopoDS_Vertex V2);
        TopoDS_Shape initEFOR = E.Oriented(TopAbs_FORWARD);                           // skl
        TopoDS_Edge EFOR = TopoDS.Edge(initEFOR /*E.Oriented(TopAbs_FORWARD)*/); // skl
        double aTolV = 0.0;
        NewParameter(V1, EFOR, ref f, ref aTolV);
        NewParameter(V2, EFOR, ref l, ref aTolV);
        GeomLib.SameRange(Precision.PConfusion(), NewC, newf, newl, f, l, ref C);

        return true;
    }

    /// <summary>The vertex's parameter on the edge, transformed with the edge's curve; false for a null vertex.</summary>
    public override bool NewParameter(in TopoDS_Vertex V, in TopoDS_Edge E, ref double P, ref double Tol)
    {
        if (V.IsNull())
        {
            return false; // infinite edge may have Null vertex
        }

        Tol = BRep_Tool.Tolerance(V);
        Tol *= Math.Abs(myTrsf.ScaleFactor());
        P = BRep_Tool.Parameter(V, E);

        Geom_Curve? C = BRep_Tool.Curve(E, out _, out _, out _);
        if (C is not null)
        {
            P = C.TransformedParameter(P, myTrsf);
        }

        return true;
    }

    /// <summary>The edge's continuity between the original faces.</summary>
    public override GeomAbs_Shape Continuity(in TopoDS_Edge E,
                                             in TopoDS_Face F1,
                                             in TopoDS_Face F2,
                                             in TopoDS_Edge NewE,
                                             in TopoDS_Face NewF1,
                                             in TopoDS_Face NewF2)
    {
        return BRep_Tool.Continuity(E, F1, F2);
    }
}
