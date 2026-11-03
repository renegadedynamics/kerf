// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepTool/TopOpeBRepTool_ShapeClassifier.hxx
// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepTool/TopOpeBRepTool_ShapeClassifier.cxx
// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepTool/TopOpeBRepTool_PShapeClassifier.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1996-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Geometry;
using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;
using static Occt.TopAbs_State;

namespace Occt;

/// <summary>
/// Classifies a shape against a reference shape (a face, a solid or a shell): through a point of one of its edges (2D
/// classification in a reference face when the two are of the same domain, 3D otherwise) or a point of a face.
/// </summary>
/// <remarks>
/// <para>
/// A working object. OCCT's default constructor leaves <c>mySameDomain</c>, <c>mymren</c>, <c>mymredone</c>,
/// <c>myState</c> and <c>myPAvLS</c> uninitialised; the port's are zero, null and false (marked <c>// OCCT</c>).
/// Every caller sets them (SetReference, SameDomain, StateShapeReference) before they are read.
/// </para>
/// <para>The list of shapes to avoid (<c>myPAvLS</c>, a pointer in OCCT) is a reference to the caller's list.</para>
/// </remarks>
public sealed class TopOpeBRepTool_ShapeClassifier
{
    private TopoDS_Shape myS = new();
    private TopoDS_Shape myRef = new();
    private TopoDS_Shape myAvS = new();
    private NCollection_List<TopoDS_Shape>? myPAvLS;
    private readonly NCollection_IndexedMap<TopoDS_Shape> myMapAvS = new(TopTools_ShapeMapHasher.Instance);
    private readonly NCollection_IndexedMap<TopoDS_Shape> mymre = new(TopTools_ShapeMapHasher.Instance);
    private int mymren;
    private bool mymredone;
    private TopAbs_State myState;
    private TopoDS_Edge myEdge = new();
    private TopoDS_Face myFace = new();
    private bool myP3Ddef;
    private Point3D myP3D = new();
    private bool myP2Ddef;
    private Point2D myP2D = new();
    private readonly TopOpeBRepTool_SolidClassifier mySolidClassifier = new();
    private int mySameDomain;

    /// <summary>A classifier without a reference shape.</summary>
    public TopOpeBRepTool_ShapeClassifier()
    {
        // OCCT: mySameDomain, mymren, mymredone, myState and myPAvLS are left uninitialised (zero, false and null here)
        myP3Ddef = false;
        myP2Ddef = false;
    }

    /// <summary>A classifier with the reference shape <paramref name="SRef"/>.</summary>
    public TopOpeBRepTool_ShapeClassifier(in TopoDS_Shape SRef)
    {
        myP3Ddef = false;
        myP2Ddef = false;
        myRef = SRef;
    }

    /// <summary>Resets everything, the solid classifier included.</summary>
    public void ClearAll()
    {
        ClearCurrent();
        mySolidClassifier.Clear();
    }

    /// <summary>Resets everything but the solid classifier.</summary>
    public void ClearCurrent()
    {
        mySameDomain = -1;
        myS.Nullify();
        myRef.Nullify();
        myAvS.Nullify();
        myMapAvS.Clear();
        mymre.Clear();
        mymren = 0;
        mymredone = false;
        myState = TopAbs_UNKNOWN;
        myEdge.Nullify();
        myFace.Nullify();
        myP3Ddef = myP2Ddef = false;
    }

    /// <summary>The same-domain mode (1: the shape and the reference are of the same domain).</summary>
    public int SameDomain() => mySameDomain;

    /// <summary>Sets the same-domain mode for the next classification.</summary>
    public void SameDomain(int sam)
    {
        mySameDomain = sam;
    }

    /// <summary>Sets the reference shape.</summary>
    public void SetReference(in TopoDS_Shape SRef)
    {
        myRef = SRef;
        MapRef();
    }

    private void MapRef()
    {
        mymre.Clear();
        mymren = 0;
        if (myRef.ShapeType() == TopAbs_FACE && mySameDomain == 1)
        {
            TopExp.MapShapes(myRef, TopAbs_EDGE, mymre);
            mymren = mymre.Extent();
            if (mymren == 1)
            {
                TopExp_Explorer x = new(myRef, TopAbs_EDGE);
                TopoDS_Edge E = TopoDS.Edge(x.Current());
                TopExp.Vertices(E, out TopoDS_Vertex v1, out TopoDS_Vertex v2);
                if (v1.IsSame(v2))
                {
                    mymren = 0;
                }
            }
        }
        mymredone = true;
    }

    /// <summary>The state of <paramref name="S"/> against <paramref name="SRef"/>.</summary>
    public TopAbs_State StateShapeShape(in TopoDS_Shape S, in TopoDS_Shape SRef, int samedomain = 0)
    {
        ClearCurrent();
        mySameDomain = samedomain;
        myS = S;
        myAvS.Nullify();
        myPAvLS = null;
        myRef = SRef;
        Perform();
        return myState;
    }

    /// <summary>The state of <paramref name="S"/> against <paramref name="SRef"/>, avoiding <paramref name="AvS"/>.</summary>
    public TopAbs_State StateShapeShape(in TopoDS_Shape S, in TopoDS_Shape AvS, in TopoDS_Shape SRef)
    {
        ClearCurrent();
        myS = S;
        myAvS = AvS;
        myPAvLS = null;
        myRef = SRef;
        Perform();
        return myState;
    }

    /// <summary>The state of <paramref name="S"/> against <paramref name="SRef"/>, avoiding the shapes of <paramref name="AvLS"/>.</summary>
    public TopAbs_State StateShapeShape(in TopoDS_Shape S, NCollection_List<TopoDS_Shape> AvLS, in TopoDS_Shape SRef)
    {
        ClearCurrent();
        myS = S;
        myAvS.Nullify();
        myPAvLS = AvLS;
        myRef = SRef;
        Perform();
        return myState;
    }

    /// <summary>The state of <paramref name="S"/> against the reference shape, avoiding <paramref name="AvS"/>.</summary>
    public TopAbs_State StateShapeReference(in TopoDS_Shape S, in TopoDS_Shape AvS)
    {
        myS = S;
        myAvS = AvS;
        myPAvLS = null;
        Perform();
        return myState;
    }

    /// <summary>The state of <paramref name="S"/> against the reference shape, avoiding the shapes of <paramref name="AvLS"/>.</summary>
    public TopAbs_State StateShapeReference(in TopoDS_Shape S, NCollection_List<TopoDS_Shape> AvLS)
    {
        myS = S;
        myAvS.Nullify();
        myPAvLS = AvLS;
        Perform();
        return myState;
    }

    /// <summary>The solid classifier.</summary>
    public TopOpeBRepTool_SolidClassifier ChangeSolidClassifier() => mySolidClassifier;

    private void FindEdge()
    {
        myEdge.Nullify();
        myFace.Nullify();

        TopAbs_ShapeEnum t = myS.ShapeType();
        if (t < TopAbs_FACE)
        { // compsolid .. shell
            FindFace(myS);
            FindEdge(myFace);
        }
        else
        {
            FindEdge(myS);
        }
    }

    private void FindEdge(in TopoDS_Shape S)
    {
        myEdge.Nullify();
        bool isavls = HasAvLS();
        bool isavs = !myAvS.IsNull();
        if (S.IsNull())
        {
            return;
        }

        TopAbs_ShapeEnum tS = S.ShapeType();
        TopExp_Explorer eex = new();
        if (!myFace.IsNull())
        {
            eex.Init(myFace, TopAbs_EDGE);
        }
        else
        {
            eex.Init(S, TopAbs_EDGE);
        }

        for (; eex.More(); eex.Next())
        {
            TopoDS_Edge E = TopoDS.Edge(eex.Current());
            bool toavoid = false;
            if (isavls || isavs)
            {
                toavoid = toavoid || myMapAvS.Contains(E);
                if (!myAvS.IsNull())
                {
                    toavoid = toavoid || E.IsSame(myAvS);
                }
            }
            else if (BRep_Tool.Degenerated(E))
            {
                toavoid = tS != TopAbs_EDGE;
            }
            if (toavoid)
            {
                continue;
            }
            myEdge = E;
            break;
        }
    }

    private void FindFace(in TopoDS_Shape S)
    {
        myFace.Nullify();
        bool isavls = HasAvLS();
        bool isavs = !myAvS.IsNull();
        TopExp_Explorer fex = new(S, TopAbs_FACE);
        for (; fex.More(); fex.Next())
        {
            TopoDS_Face F = TopoDS.Face(fex.Current());
            bool toavoid = false;
            if (isavls || isavs)
            {
                toavoid = toavoid || myMapAvS.Contains(F);
                if (!myAvS.IsNull())
                {
                    toavoid = toavoid || F.IsSame(myAvS);
                }
            }
            if (toavoid)
            {
                continue;
            }
            myFace = F;
            break;
        }
    }

    private void Perform()
    {
        myState = TopAbs_UNKNOWN;
        if (myS.IsNull())
        {
            return;
        }
        if (myRef.IsNull())
        {
            return;
        }

        if (!mymredone)
        {
            MapRef();
        }

        if (!myAvS.IsNull())
        {
            // tAvS = FACE,EDGE --> map(AvS,EDGE)
            // rejection des aretes de myS comme arete de classification
            // (le point de classification doit etre IN S)
            myMapAvS.Clear();
            TopAbs_ShapeEnum tAvS = myAvS.ShapeType();
            if (tAvS == TopAbs_FACE)
            {
                myMapAvS.Add(myAvS);
                TopExp.MapShapes(myAvS, TopAbs_EDGE, myMapAvS);
            }
            else if (tAvS == TopAbs_EDGE)
            {
                TopExp.MapShapes(myAvS, TopAbs_EDGE, myMapAvS);
            }
        }
        else if (HasAvLS())
        {
            // tAvS = FACE,EDGE --> map(AvS,EDGE)
            myMapAvS.Clear();
            TopAbs_ShapeEnum tAvS = myPAvLS!.First().ShapeType();
            if (tAvS == TopAbs_FACE)
            {
                NCollection_List<TopoDS_Shape>.Iterator it = new(myPAvLS);
                for (; it.More(); it.Next())
                {
                    TopoDS_Shape S = it.Value();
                    myMapAvS.Add(S);
                    TopExp.MapShapes(S, TopAbs_EDGE, myMapAvS);
                }
            }
            else if (tAvS == TopAbs_EDGE)
            {
                NCollection_List<TopoDS_Shape>.Iterator it = new(myPAvLS);
                for (; it.More(); it.Next())
                {
                    TopoDS_Shape S = it.Value();
                    TopExp.MapShapes(S, TopAbs_EDGE, myMapAvS);
                }
            }
        }
        else
        {
            if (myS.ShapeType() == TopAbs_FACE)
            {
                myP3Ddef = BRepClass3d_SolidExplorer.FindAPointInTheFace(TopoDS.Face(myS), ref myP3D);
            }
        }

        TopAbs_ShapeEnum tS = myS.ShapeType();
        TopAbs_ShapeEnum tR = myRef.ShapeType();

        if (tS == TopAbs_VERTEX)
        {
            if (tR <= TopAbs_SOLID)
            {
                Point3D P3D = BRep_Tool.Pnt(TopoDS.Vertex(myS));
                StateP3DReference(P3D);
            }
        }
        else if (tS == TopAbs_EDGE)
        {
            if (tR == TopAbs_FACE || tR <= TopAbs_SOLID)
            {
                FindEdge();
                StateEdgeReference();
            }
        }
        else if (tS == TopAbs_WIRE)
        {
            if (tR == TopAbs_FACE || tR <= TopAbs_SOLID)
            {
                FindEdge();
                StateEdgeReference();
            }
        }
        else if (tS == TopAbs_FACE)
        {
            if (tR == TopAbs_FACE)
            {
                FindEdge();
                if (mySameDomain == 1)
                {
                    StateEdgeReference();
                }
                else
                {
                    if (!myP3Ddef)
                    {
                        myP3Ddef = BRepClass3d_SolidExplorer.FindAPointInTheFace(TopoDS.Face(myS), ref myP3D);
                    }
                    if (myP3Ddef)
                    {
                        StateP3DReference(myP3D);
                    }
                    else
                    {
                        myState = TopAbs_UNKNOWN;
                        throw new ProgramErrorException("TopOpeBRepTool_ShapeClassifier !P3Ddef");
                    }
                }
            }
            else if (tR <= TopAbs_SOLID)
            {
                FindEdge();
                if (myP3Ddef)
                {
                    StateP3DReference(myP3D);
                }
                else
                {
                    StateEdgeReference();
                }
            }
        }
        else if (tS == TopAbs_SHELL)
        {
            if (tR <= TopAbs_SOLID)
            {
                FindEdge();
                StateEdgeReference();
            }
        }
        else if (tS == TopAbs_SOLID)
        {
            if (tR <= TopAbs_SOLID)
            {
                FindEdge();
                StateEdgeReference();
            }
        }
        else
        {
            throw new ProgramErrorException("StateShapeShape : bad operands");
        }

        // take orientation of reference shape in account
        TopAbs_Orientation oriRef = myRef.Orientation();
        if (oriRef == TopAbs_EXTERNAL || oriRef == TopAbs_INTERNAL)
        {
            if (myState == TopAbs_IN)
            {
                myState = TopAbs_OUT;
            }
        }
    }

    private void StateEdgeReference()
    {
        myState = TopAbs_UNKNOWN;

        if (myEdge.IsNull())
        {
            return;
        }
        if (myRef.IsNull())
        {
            return;
        }

        Geom_Curve? C3D;
        Point3D P3D;
        Geom2d_Curve? C2D;
        Point2D P2D;

        TopAbs_ShapeEnum tR = myRef.ShapeType();

        if (tR == TopAbs_FACE)
        {
            TopoDS_Face F = TopoDS.Face(myRef);
            if (mySameDomain != 0)
            {
                bool trimCurve = true;
                C2D = TopOpeBRepTool_2d.FC2D_CurveOnSurface(myEdge, F, out double f2d, out double l2d, out _, trimCurve);

                if (C2D is null)
                {
                    throw new ProgramErrorException("StateShapeShape : no 2d curve");
                }

                double t = 0.127956477;
                double p = (1 - t) * f2d + t * l2d;
                P2D = C2D.Value(p);

                StateP2DReference(P2D);
                return;
            }
            else
            { // myEdge/myRef=face en 3d
                C3D = BRep_Tool.Curve(myEdge, out double f3d, out double l3d);

                if (C3D is null)
                {
                    throw new ProgramErrorException("StateShapeShape : no 3d curve");
                }

                double t = 0.127956477;
                double p = (1 - t) * f3d + t * l3d;
                P3D = C3D.Value(p);
                StateP3DReference(P3D);
                return;
            }
        }
        else if (tR <= TopAbs_SOLID)
        {
            bool degen = BRep_Tool.Degenerated(myEdge);
            if (degen)
            {
                TopoDS_Vertex v = TopExp.FirstVertex(myEdge);
                P3D = BRep_Tool.Pnt(v);
                StateP3DReference(P3D);
                return;
            }
            else
            {
                C3D = BRep_Tool.Curve(myEdge, out double f3d, out double l3d);

                if (C3D is null)
                {
                    throw new ProgramErrorException("StateShapeShape : no 3d curve");
                }

                double t = 0.127956477;
                double p = (1 - t) * f3d + t * l3d;
                P3D = C3D.Value(p);
                StateP3DReference(P3D);
                return;
            }
        }
        else
        {
            throw new ProgramErrorException("StateShapeShape : bad operands");
        }
    }

    /// <summary>Classifies the point <paramref name="P2D"/> in the reference face.</summary>
    public void StateP2DReference(in Point2D P2D)
    {
        myState = TopAbs_UNKNOWN;
        if (myRef.IsNull())
        {
            return;
        }
        TopAbs_ShapeEnum tR = myRef.ShapeType();

        if (tR == TopAbs_FACE)
        {
            if (mymren == 1)
            {
                TopExp_Explorer x = new();
                for (x.Init(myRef, TopAbs_EDGE); x.More(); x.Next())
                {
                    TopAbs_Orientation o = x.Current().Orientation();
                    if (o == TopAbs_EXTERNAL)
                    {
                        myState = TopAbs_OUT;
                    }
                    else if (o == TopAbs_INTERNAL)
                    {
                        myState = TopAbs_IN;
                    }
                    else
                    {
                        break;
                    }
                }
            }
            else
            {
                myP2D = P2D;
                myP2Ddef = true;
                TopoDS_Face F = TopoDS.Face(myRef);
                F.Orientation(TopAbs_FORWARD);
                double TolClass = 1e-8;
                BRepTopAdaptor_FClass2d FClass2d = new(F, TolClass);
                myState = FClass2d.Perform(P2D);
            }
        }
        else
        {
            throw new ProgramErrorException("StateShapeShape : bad operands");
        }
    }

    /// <summary>Classifies the point <paramref name="P3D"/> in the reference solid (or the solids of the reference).</summary>
    public void StateP3DReference(in Point3D P3D)
    {
        myState = TopAbs_UNKNOWN;
        if (myRef.IsNull())
        {
            return;
        }
        TopAbs_ShapeEnum tR = myRef.ShapeType();

        if (tR == TopAbs_SOLID)
        {
            myP3D = P3D;
            myP3Ddef = true;
            TopoDS_Solid SO = TopoDS.Solid(myRef);
            double tol3d = global::Kerf.Engine.Foundation.Precision.Confusion();
            mySolidClassifier.Classify(SO, P3D, tol3d);
            myState = mySolidClassifier.State();
        }
        else if (tR < TopAbs_SOLID)
        {
            myP3D = P3D;
            myP3Ddef = true;
            TopExp_Explorer ex = new();
            for (ex.Init(myRef, TopAbs_SOLID); ex.More(); ex.Next())
            {
                TopoDS_Solid SO = TopoDS.Solid(ex.Current());
                double tol3d = global::Kerf.Engine.Foundation.Precision.Confusion();
                mySolidClassifier.Classify(SO, P3D, tol3d);
                myState = mySolidClassifier.State();
                if (myState == TopAbs_IN || myState == TopAbs_ON)
                {
                    break;
                }
            }
        }
        else
        {
            throw new ProgramErrorException("StateShapeShape : bad operands");
        }
    }

    /// <summary>The state found by the last classification.</summary>
    public TopAbs_State State() => myState;

    /// <summary>The 3D point used by the last classification.</summary>
    /// <exception cref="ProgramErrorException">There is none.</exception>
    public Point3D P3D()
    {
        if (myP3Ddef)
        {
            return myP3D;
        }
        throw new ProgramErrorException("ShapeClassifier::P3D undefined");
    }

    /// <summary>The 2D point used by the last classification.</summary>
    /// <exception cref="ProgramErrorException">There is none.</exception>
    public Point2D P2D()
    {
        if (myP2Ddef)
        {
            return myP2D;
        }
        throw new ProgramErrorException("ShapeClassifier::P2D undefined");
    }

    private bool HasAvLS()
    {
        bool hasavls = myPAvLS is not null && !myPAvLS.IsEmpty();
        return hasavls;
    }
}
