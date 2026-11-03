// Port of: src/ModelingData/TKBRep/BRepTools/BRepTools_TrsfModification.hxx
// Port of: src/ModelingData/TKBRep/BRepTools/BRepTools_TrsfModification.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Meshing;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Geometry.SurfaceKind;
using static Kerf.Geometry.TransformForm;
using static Occt.TopAbs_Orientation;

namespace Occt;

/// <summary>
/// The modification that applies a transformation (gp_Trsf) to the geometry of a shape: surfaces, curves, points,
/// pcurves (reparametrised where the transformation changes the parametrisation) and, if <see cref="IsCopyMesh"/>,
/// triangulations and polygons.
/// </summary>
public class BRepTools_TrsfModification : BRepTools_Modification
{
    private Transform3D myTrsf;
    private bool myCopyMesh;

    /// <summary>The modification applying <paramref name="T"/>; meshes are not copied.</summary>
    public BRepTools_TrsfModification(in Transform3D T)
    {
        myTrsf = T;
        myCopyMesh = false;
    }

    /// <summary>The transformation, for reading and writing (a working object).</summary>
    public ref Transform3D Trsf() => ref myTrsf;

    /// <summary>Whether triangulations and polygons are transformed too, for reading and writing.</summary>
    public ref bool IsCopyMesh() => ref myCopyMesh;

    /// <summary>The face's surface transformed (in the frame of its location); false if the face has no surface.</summary>
    public override bool NewSurface(in TopoDS_Face F,
                                    out Geom_Surface? S,
                                    out Placement L,
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
        RevFace = myTrsf.IsNegative;

        Transform3D LT = L.Transformation();
        LT.Invert();
        LT.Multiply(myTrsf);
        LT.Multiply(L.Transformation());

        S = (Geom_Surface)S.Transformed(LT);

        return true;
    }

    /// <summary>The face's triangulation transformed, if meshes are copied.</summary>
    public override bool NewTriangulation(in TopoDS_Face theFace, out Triangulation? theTriangulation)
    {
        theTriangulation = null;
        if (!myCopyMesh)
        {
            return false;
        }

        theTriangulation = BRep_Tool.Triangulation(theFace, out Placement aLoc);

        if (theTriangulation is null)
        {
            return false;
        }

        Transform3D aTrsf = myTrsf;
        if (!aLoc.IsIdentity)
        {
            aTrsf = aLoc.Transformation().Inverted() * aTrsf * aLoc.Transformation();
        }

        theTriangulation = theTriangulation.Copy();
        theTriangulation.SetCachedMinMax(new Box()); // clear bounding box
        theTriangulation.Deflection(theTriangulation.Deflection() * Math.Abs(myTrsf.ScaleFactor()));
        // apply transformation to 3D nodes
        for (int anInd = 1; anInd <= theTriangulation.NodeCount; ++anInd)
        {
            Point3D aP = theTriangulation.Node(anInd);
            aP.Transform(aTrsf);
            theTriangulation.SetNode(anInd, aP);
        }
        // modify 2D nodes
        Geom_Surface? aSurf = BRep_Tool.Surface(theFace, out aLoc);
        if (theTriangulation.HasUVNodes && aSurf is not null)
        {
            for (int anInd = 1; anInd <= theTriangulation.NodeCount; ++anInd)
            {
                Point2D aP2d = theTriangulation.UVNode(anInd);
                double aU = aP2d.X(), aV = aP2d.Y();
                aSurf.TransformParameters(ref aU, ref aV, myTrsf);
                aP2d.SetCoord(aU, aV);
                theTriangulation.SetUVNode(anInd, aP2d);
            }
        }
        // modify triangles orientation in case of mirror transformation
        if (myTrsf.ScaleFactor() < 0.0)
        {
            for (int anInd = 1; anInd <= theTriangulation.TriangleCount; ++anInd)
            {
                Triangle1 aTria = theTriangulation.Triangle(anInd);
                aTria.Get(out int aN1, out int aN2, out int aN3);
                aTria.Set(aN1, aN3, aN2);
                theTriangulation.SetTriangle(anInd, aTria);
            }
        }
        // modify normals
        if (theTriangulation.HasNormals)
        {
            for (int anInd = 1; anInd <= theTriangulation.NodeCount; ++anInd)
            {
                Direction3D aNormal = theTriangulation.Normal(anInd);
                aNormal.Transform(aTrsf);
                theTriangulation.SetNormal(anInd, aNormal);
            }
        }

        return true;
    }

    /// <summary>The edge's 3D polygon transformed (its parameters too), if meshes are copied.</summary>
    public override bool NewPolygon(in TopoDS_Edge theE, out Polygon3D? theP)
    {
        theP = null;
        if (!myCopyMesh)
        {
            return false;
        }

        theP = BRep_Tool.Polygon3D(theE, out Placement aLoc);
        if (theP is null)
        {
            return false;
        }

        Transform3D aTrsf = myTrsf;
        if (!aLoc.IsIdentity)
        {
            aTrsf = aLoc.Transformation().Inverted() * aTrsf * aLoc.Transformation();
        }

        theP = theP.Copy();
        theP.Deflection(theP.Deflection() * Math.Abs(myTrsf.ScaleFactor()));
        NCollection_Array1<Point3D> aNodesArray = theP.ChangeNodes();
        for (int anId = aNodesArray.Lower(); anId <= aNodesArray.Upper(); ++anId)
        {
            // Applying the transformation to each node of polygon
            aNodesArray[anId].Transform(aTrsf);
        }
        // transform the parametrization
        if (theP.HasParameters)
        {
            Geom_Curve? aCurve = BRep_Tool.Curve(theE, out _, out _, out _);
            if (aCurve is not null)
            {
                double aReparametrization = aCurve.ParametricTransformation(aTrsf);
                if (Math.Abs(aReparametrization - 1.0) > global::Kerf.Engine.Foundation.Precision.PConfusion())
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
    public override bool NewPolygonOnTriangulation(in TopoDS_Edge theE, in TopoDS_Face theF, out PolygonOnTriangulation? theP)
    {
        theP = null;
        if (!myCopyMesh)
        {
            return false;
        }

        Triangulation? aT = BRep_Tool.Triangulation(theF, out Placement aLoc);
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
            && Math.Abs(Math.Abs(myTrsf.ScaleFactor()) - 1.0) > Placement.ScalePrec())
        {
            GeneralTransform2D aGTrsf = aSurf.ParametricTransformation(myTrsf);
            if (aGTrsf.Form() != Identity)
            {
                Geom2d_Curve? aNewC2d = GeomLib.GTransform(aC2d, aGTrsf);
                for (int anInd = 1; anInd <= theP.NodeCount; ++anInd)
                {
                    double aParam = theP.Parameter(anInd);
                    Point2D aP2d = aC2d.Value(aParam);
                    Xy aXY = aP2d.XY();
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
    public override bool NewCurve(in TopoDS_Edge E, out Geom_Curve? C, out Placement L, out double Tol)
    {
        C = BRep_Tool.Curve(E, out L, out _, out _);

        Tol = BRep_Tool.Tolerance(E);
        Tol *= Math.Abs(myTrsf.ScaleFactor());

        Transform3D LT = L.Transformation();
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
    public override bool NewPoint(in TopoDS_Vertex V, out Point3D P, out double Tol)
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
        if (GAsurf.GetType() == Plane)
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
            if (fc - f > global::Kerf.Engine.Foundation.Precision.PConfusion())
            {
                f = fc;
            }
            if (l - lc > global::Kerf.Engine.Foundation.Precision.PConfusion())
            {
                l = lc;
            }
            if (Math.Abs(l - f) < global::Kerf.Engine.Foundation.Precision.PConfusion())
            {
                if (Math.Abs(f - fc) < global::Kerf.Engine.Foundation.Precision.PConfusion() && !global::Kerf.Engine.Foundation.Precision.IsInfinite(lc))
                {
                    l = lc;
                }
                else if (!global::Kerf.Engine.Foundation.Precision.IsInfinite(fc))
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
            GeneralTransform2D gtrsf = S.ParametricTransformation(myTrsf);

            if (gtrsf.Form() != Identity)
            {
                NewC = GeomLib.GTransform(NewC, gtrsf);
                if (NewC is null)
                {
                    throw new DomainErrorException("TrsfModification:Error in NewCurve2d");
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
        GeomLib.SameRange(global::Kerf.Engine.Foundation.Precision.PConfusion(), NewC, newf, newl, f, l, ref C);

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
    public override Continuity Continuity(in TopoDS_Edge E,
                                             in TopoDS_Face F1,
                                             in TopoDS_Face F2,
                                             in TopoDS_Edge NewE,
                                             in TopoDS_Face NewF1,
                                             in TopoDS_Face NewF2)
    {
        return BRep_Tool.Continuity(E, F1, F2);
    }
}
