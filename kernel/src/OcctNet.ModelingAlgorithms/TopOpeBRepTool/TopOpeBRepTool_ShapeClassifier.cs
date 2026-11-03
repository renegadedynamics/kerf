// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepTool/TopOpeBRepTool_ShapeClassifier.hxx
// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepTool/TopOpeBRepTool_ShapeClassifier.cxx
// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepTool/TopOpeBRepTool_PShapeClassifier.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1996-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;
using static Kerf.Topology.Containment;

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
    private Shape myS = new();
    private Shape myRef = new();
    private Shape myAvS = new();
    private NCollection_List<Shape>? myPAvLS;
    private readonly NCollection_IndexedMap<Shape> myMapAvS = new(ShapeHasher.Instance);
    private readonly NCollection_IndexedMap<Shape> mymre = new(ShapeHasher.Instance);
    private int mymren;
    private bool mymredone;
    private Containment myState;
    private Edge myEdge = new();
    private Face myFace = new();
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
    public TopOpeBRepTool_ShapeClassifier(in Shape SRef)
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
        myState = Unknown;
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
    public void SetReference(in Shape SRef)
    {
        myRef = SRef;
        MapRef();
    }

    private void MapRef()
    {
        mymre.Clear();
        mymren = 0;
        if (myRef.ShapeType() == ShapeType.Face && mySameDomain == 1)
        {
            Exploration.MapShapes(myRef, ShapeType.Edge, mymre);
            mymren = mymre.Extent();
            if (mymren == 1)
            {
                Explorer x = new(myRef, ShapeType.Edge);
                Edge E = ShapeCasts.Edge(x.Current());
                Exploration.Vertices(E, out Vertex v1, out Vertex v2);
                if (v1.IsSame(v2))
                {
                    mymren = 0;
                }
            }
        }
        mymredone = true;
    }

    /// <summary>The state of <paramref name="S"/> against <paramref name="SRef"/>.</summary>
    public Containment StateShapeShape(in Shape S, in Shape SRef, int samedomain = 0)
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
    public Containment StateShapeShape(in Shape S, in Shape AvS, in Shape SRef)
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
    public Containment StateShapeShape(in Shape S, NCollection_List<Shape> AvLS, in Shape SRef)
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
    public Containment StateShapeReference(in Shape S, in Shape AvS)
    {
        myS = S;
        myAvS = AvS;
        myPAvLS = null;
        Perform();
        return myState;
    }

    /// <summary>The state of <paramref name="S"/> against the reference shape, avoiding the shapes of <paramref name="AvLS"/>.</summary>
    public Containment StateShapeReference(in Shape S, NCollection_List<Shape> AvLS)
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

        ShapeType t = myS.ShapeType();
        if (t < ShapeType.Face)
        { // compsolid .. shell
            FindFace(myS);
            FindEdge(myFace);
        }
        else
        {
            FindEdge(myS);
        }
    }

    private void FindEdge(in Shape S)
    {
        myEdge.Nullify();
        bool isavls = HasAvLS();
        bool isavs = !myAvS.IsNull;
        if (S.IsNull)
        {
            return;
        }

        ShapeType tS = S.ShapeType();
        Explorer eex = new();
        if (!myFace.IsNull)
        {
            eex.Init(myFace, ShapeType.Edge);
        }
        else
        {
            eex.Init(S, ShapeType.Edge);
        }

        for (; eex.More(); eex.Next())
        {
            Edge E = ShapeCasts.Edge(eex.Current());
            bool toavoid = false;
            if (isavls || isavs)
            {
                toavoid = toavoid || myMapAvS.Contains(E);
                if (!myAvS.IsNull)
                {
                    toavoid = toavoid || E.IsSame(myAvS);
                }
            }
            else if (Tool.Degenerated(E))
            {
                toavoid = tS != ShapeType.Edge;
            }
            if (toavoid)
            {
                continue;
            }
            myEdge = E;
            break;
        }
    }

    private void FindFace(in Shape S)
    {
        myFace.Nullify();
        bool isavls = HasAvLS();
        bool isavs = !myAvS.IsNull;
        Explorer fex = new(S, ShapeType.Face);
        for (; fex.More(); fex.Next())
        {
            Face F = ShapeCasts.Face(fex.Current());
            bool toavoid = false;
            if (isavls || isavs)
            {
                toavoid = toavoid || myMapAvS.Contains(F);
                if (!myAvS.IsNull)
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
        myState = Unknown;
        if (myS.IsNull)
        {
            return;
        }
        if (myRef.IsNull)
        {
            return;
        }

        if (!mymredone)
        {
            MapRef();
        }

        if (!myAvS.IsNull)
        {
            // tAvS = FACE,EDGE --> map(AvS,EDGE)
            // rejection des aretes de myS comme arete de classification
            // (le point de classification doit etre IN S)
            myMapAvS.Clear();
            ShapeType tAvS = myAvS.ShapeType();
            if (tAvS == ShapeType.Face)
            {
                myMapAvS.Add(myAvS);
                Exploration.MapShapes(myAvS, ShapeType.Edge, myMapAvS);
            }
            else if (tAvS == ShapeType.Edge)
            {
                Exploration.MapShapes(myAvS, ShapeType.Edge, myMapAvS);
            }
        }
        else if (HasAvLS())
        {
            // tAvS = FACE,EDGE --> map(AvS,EDGE)
            myMapAvS.Clear();
            ShapeType tAvS = myPAvLS!.First().ShapeType();
            if (tAvS == ShapeType.Face)
            {
                NCollection_List<Shape>.Iterator it = new(myPAvLS);
                for (; it.More(); it.Next())
                {
                    Shape S = it.Value();
                    myMapAvS.Add(S);
                    Exploration.MapShapes(S, ShapeType.Edge, myMapAvS);
                }
            }
            else if (tAvS == ShapeType.Edge)
            {
                NCollection_List<Shape>.Iterator it = new(myPAvLS);
                for (; it.More(); it.Next())
                {
                    Shape S = it.Value();
                    Exploration.MapShapes(S, ShapeType.Edge, myMapAvS);
                }
            }
        }
        else
        {
            if (myS.ShapeType() == ShapeType.Face)
            {
                myP3Ddef = BRepClass3d_SolidExplorer.FindAPointInTheFace(ShapeCasts.Face(myS), ref myP3D);
            }
        }

        ShapeType tS = myS.ShapeType();
        ShapeType tR = myRef.ShapeType();

        if (tS == ShapeType.Vertex)
        {
            if (tR <= ShapeType.Solid)
            {
                Point3D P3D = Tool.Pnt(ShapeCasts.Vertex(myS));
                StateP3DReference(P3D);
            }
        }
        else if (tS == ShapeType.Edge)
        {
            if (tR == ShapeType.Face || tR <= ShapeType.Solid)
            {
                FindEdge();
                StateEdgeReference();
            }
        }
        else if (tS == ShapeType.Wire)
        {
            if (tR == ShapeType.Face || tR <= ShapeType.Solid)
            {
                FindEdge();
                StateEdgeReference();
            }
        }
        else if (tS == ShapeType.Face)
        {
            if (tR == ShapeType.Face)
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
                        myP3Ddef = BRepClass3d_SolidExplorer.FindAPointInTheFace(ShapeCasts.Face(myS), ref myP3D);
                    }
                    if (myP3Ddef)
                    {
                        StateP3DReference(myP3D);
                    }
                    else
                    {
                        myState = Unknown;
                        throw new ProgramErrorException("TopOpeBRepTool_ShapeClassifier !P3Ddef");
                    }
                }
            }
            else if (tR <= ShapeType.Solid)
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
        else if (tS == ShapeType.Shell)
        {
            if (tR <= ShapeType.Solid)
            {
                FindEdge();
                StateEdgeReference();
            }
        }
        else if (tS == ShapeType.Solid)
        {
            if (tR <= ShapeType.Solid)
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
        Orientation oriRef = myRef.Orientation();
        if (oriRef == External || oriRef == Internal)
        {
            if (myState == In)
            {
                myState = Out;
            }
        }
    }

    private void StateEdgeReference()
    {
        myState = Unknown;

        if (myEdge.IsNull)
        {
            return;
        }
        if (myRef.IsNull)
        {
            return;
        }

        Curve3D? C3D;
        Point3D P3D;
        Curve2D? C2D;
        Point2D P2D;

        ShapeType tR = myRef.ShapeType();

        if (tR == ShapeType.Face)
        {
            Face F = ShapeCasts.Face(myRef);
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
                C3D = Tool.Curve(myEdge, out double f3d, out double l3d);

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
        else if (tR <= ShapeType.Solid)
        {
            bool degen = Tool.Degenerated(myEdge);
            if (degen)
            {
                Vertex v = Exploration.FirstVertex(myEdge);
                P3D = Tool.Pnt(v);
                StateP3DReference(P3D);
                return;
            }
            else
            {
                C3D = Tool.Curve(myEdge, out double f3d, out double l3d);

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
        myState = Unknown;
        if (myRef.IsNull)
        {
            return;
        }
        ShapeType tR = myRef.ShapeType();

        if (tR == ShapeType.Face)
        {
            if (mymren == 1)
            {
                Explorer x = new();
                for (x.Init(myRef, ShapeType.Edge); x.More(); x.Next())
                {
                    Orientation o = x.Current().Orientation();
                    if (o == External)
                    {
                        myState = Out;
                    }
                    else if (o == Internal)
                    {
                        myState = In;
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
                Face F = ShapeCasts.Face(myRef);
                F.Orientation(Forward);
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
        myState = Unknown;
        if (myRef.IsNull)
        {
            return;
        }
        ShapeType tR = myRef.ShapeType();

        if (tR == ShapeType.Solid)
        {
            myP3D = P3D;
            myP3Ddef = true;
            Solid SO = ShapeCasts.Solid(myRef);
            double tol3d = global::Kerf.Engine.Foundation.Precision.Confusion();
            mySolidClassifier.Classify(SO, P3D, tol3d);
            myState = mySolidClassifier.State();
        }
        else if (tR < ShapeType.Solid)
        {
            myP3D = P3D;
            myP3Ddef = true;
            Explorer ex = new();
            for (ex.Init(myRef, ShapeType.Solid); ex.More(); ex.Next())
            {
                Solid SO = ShapeCasts.Solid(ex.Current());
                double tol3d = global::Kerf.Engine.Foundation.Precision.Confusion();
                mySolidClassifier.Classify(SO, P3D, tol3d);
                myState = mySolidClassifier.State();
                if (myState == In || myState == On)
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
    public Containment State() => myState;

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
