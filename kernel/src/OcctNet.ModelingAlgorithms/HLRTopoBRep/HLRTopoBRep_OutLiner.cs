// Port of: src/ModelingAlgorithms/TKHLR/HLRTopoBRep/HLRTopoBRep_OutLiner.hxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRTopoBRep/HLRTopoBRep_OutLiner.lxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRTopoBRep/HLRTopoBRep_OutLiner.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Extrema;
using Kerf.Engine.Foundation;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Geometry.CurveKind;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Occt;

/// <summary>
/// A shape and its outlined shape for a projector: each face rebuilt with its edges split at the outline vertices,
/// its internal outlines (silhouettes) and its isolines as internal wires, with the data that maps them back.
/// </summary>
/// <remarks>
/// <para>A working object (a transient), as OCCT's.</para>
/// <para>
/// OCCT's outlining writes into the original shape: the vertices of the edges it splits get point representations on
/// the new edges (HLRTopoBRep_DSFiller's <c>ProcessEdges</c>, HLRTopoBRep_FaceIsoLiner's <c>MakeIsoLine</c>). So
/// <see cref="Fill"/> copies an original shape holding frozen TShapes at the entry (<c>BRepTools.ExactCopyForWrite</c>,
/// decision 1095, as decisions 973, 1005 and 1013) and outlines the copy, its working shape; an unfrozen shape (a
/// ported caller's) is outlined in place, as OCCT does. <see cref="OriginalShape()"/> stays the caller's shape; the
/// outlined shape, the data and the maps of adaptors given to <see cref="Fill"/> refer to the working shape, which
/// ported code reads with <see cref="InternalWorkingShape"/> (HLRBRep_ShapeToHLR). The hidden-line results are new
/// edges, so nothing is to be given back. <see cref="OutLinedShape()"/> is a publication point (frozen);
/// <see cref="InternalOutLinedShape"/> doesn't freeze.
/// </para>
/// </remarks>
public class HLRTopoBRep_OutLiner : Transient
{
    private Shape myOriginalShape = new();
    private Shape myOutLinedShape = new();
    private Shape myWorkingShape = new();
    private Dictionary<TShape, TShape>? myCopies; // the original's TShapes to the working shape's (7Q2)
    private readonly HLRTopoBRep_Data myDS = new();

    public HLRTopoBRep_OutLiner()
    {
    }

    public HLRTopoBRep_OutLiner(in Shape OriSh)
    {
        myOriginalShape = OriSh;
    }

    public HLRTopoBRep_OutLiner(in Shape OriS, in Shape OutS)
    {
        myOriginalShape = OriS;
        myOutLinedShape = OutS;
    }

    public void OriginalShape(in Shape OriS) => myOriginalShape = OriS;

    /// <summary>The caller's original shape.</summary>
    public Shape OriginalShape() => myOriginalShape;

    public void OutLinedShape(in Shape OutS) => myOutLinedShape = OutS;

    /// <summary>The outlined shape (frozen).</summary>
    public Shape OutLinedShape()
    {
        myOutLinedShape.TShape()?.FreezeGraph();
        return myOutLinedShape;
    }

    /// <summary><see cref="OutLinedShape()"/> without freezing, for ported code.</summary>
    internal Shape InternalOutLinedShape() => myOutLinedShape;

    /// <summary>
    /// The shape the outlining works on: the original shape, or its copy when it held frozen TShapes (decision 1095).
    /// The original shape before <see cref="Fill"/>.
    /// </summary>
    internal Shape InternalWorkingShape() => myWorkingShape.IsNull ? myOriginalShape : myWorkingShape;

    /// <summary>
    /// The image of a sub-shape of the original shape in the working shape (the same place and orientation, the
    /// copy's TShape); the shape itself when the original wasn't copied or the shape isn't one of its sub-shapes
    /// (unit 7Q2: HLRBRep_HLRToShape finds a caller's sub-shapes in the data structure through it).
    /// </summary>
    internal Shape InternalWorkingImage(in Shape S)
    {
        if (myCopies is not null && !S.IsNull && myCopies.TryGetValue(S.TShape()!, out TShape? anImage))
        {
            Shape aShape = S;
            aShape.TShape(anImage);
            return aShape;
        }
        return S;
    }

    public HLRTopoBRep_Data DataStructure() => myDS;

    /// <summary>
    /// Builds the outlined shape for the projector P (when the original shape is set and the outlined shape is not),
    /// with nbIso isolines per face direction; MST caches the faces' adaptors.
    /// </summary>
    public void Fill(in HLRAlgo_Projector P, NCollection_DataMap<Shape, BRepTopAdaptor_Tool> MST, int nbIso)
    {
        if (!myOriginalShape.IsNull)
        {
            if (myOutLinedShape.IsNull)
            {
                // copy at the entry (decision 1095)
                myCopies = null;
                myWorkingShape = BrepTools.HoldsFrozen(myOriginalShape)
                                     ? BrepTools.ExactCopyForWrite(myOriginalShape, out _, out myCopies)
                                     : myOriginalShape;

                Vector3D Vecz = new(0.0, 0.0, 1.0);
                Transform3D Tr = P.Transformation();
                Tr.Invert();
                Vecz.Transform(Tr);
                Contap_Contour FO = new();
                if (P.Perspective())
                {
                    Point3D Eye = new();
                    Eye.SetXYZ(P.Focus() * Vecz.XYZ());
                    FO.Init(Eye);
                }
                else
                {
                    Direction3D DirZ = new(Vecz);
                    FO.Init(new Vector3D(DirZ));
                }
                HLRTopoBRep_DSFiller.Insert(myWorkingShape, FO, myDS, MST, nbIso);
                BuildShape(MST);
            }
        }
    }

    /// <summary>Builds the outlined face of F in S.</summary>
    private void ProcessFace(in Face F, ref Shape S,
                             NCollection_DataMap<Shape, BRepTopAdaptor_Tool> MST)
    {
        BrepBuilder B = new();
        Explorer exE = new(), exW = new();

        NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> aVEMap =
            new(ShapeHasher.Instance);
        Exploration.MapShapesAndAncestors(F, ShapeType.Vertex, ShapeType.Edge, aVEMap);

        Shape NF; // = F;
        NF = F.EmptyCopied();

        for (exW.Init(F, ShapeType.Wire); exW.More(); exW.Next())
        {
            Wire W = new();
            B.MakeWire(ref W);

            for (exE.Init(exW.Current(), ShapeType.Edge); exE.More(); exE.Next())
            {
                Edge E = ShapeCasts.Edge(exE.Current());
                if (myDS.EdgeHasSplE(E))
                {
                    NCollection_List<Shape>.Iterator itS = new();
                    for (itS.Initialize(myDS.EdgeSplE(E)); itS.More(); itS.Next())
                    {
                        Edge newE = ShapeCasts.Edge(itS.Value());
                        newE.Orientation(E.Orientation());
                        myDS.AddOldS(newE, E);
                        B.Add(W, newE);
                    }
                }
                else
                {
                    B.Add(W, E);
                }
            }
            B.Add(NF, W); // add the new wire in the new face.
        }
        myDS.AddIntL(F);
        NCollection_List<Shape> OutL = myDS.AddOutL(F);

        if (myDS.FaceHasIntL(F))
        { // get the InternalOutLines on face F
            Wire W = new();
            NCollection_List<Shape>.Iterator itE = new();
            for (itE.Initialize(myDS.FaceIntL(F)); itE.More(); itE.Next())
            {
                Edge E = ShapeCasts.Edge(itE.Value());
                E.Orientation(Internal);

                // Check, if outline edge coincides with face boundary
                EdgeCurveAdaptor C = new(E);
                double par = 0.34 * C.FirstParameter() + 0.66 * C.LastParameter();
                Point3D P = C.Value(par);
                Vertex V1, V2, aV1, aV2;
                Exploration.Vertices(E, out V1, out V2);
                bool SameEdge = false;
                if (!V1.IsNull && aVEMap.Contains(V1))
                {
                    NCollection_List<Shape> aEList = aVEMap.FindFromKey(V1);
                    NCollection_List<Shape>.Iterator it = new(aEList);
                    for (; it.More(); it.Next())
                    {
                        Edge aE = ShapeCasts.Edge(it.Value());
                        Exploration.Vertices(aE, out aV1, out aV2);

                        if ((V1.IsSame(aV1) && V2.IsSame(aV2)) || (V1.IsSame(aV2) && V2.IsSame(aV1)))
                        {
                            EdgeCurveAdaptor aC = new(aE);
                            if ((C.GetType() == Line) && (aC.GetType() == Line))
                            {
                                SameEdge = true;
                                break;
                            }
                            else
                            {
                                CurvePointExtrema anExt = new(P, aC);
                                if (anExt.IsDone)
                                {
                                    int aNe = anExt.ExtCount;
                                    if (aNe > 0)
                                    {
                                        double dist = RealLast();
                                        int ec;
                                        for (ec = 1; ec <= aNe; ++ec)
                                        {
                                            dist = Min(dist, anExt.SquareDistance(ec));
                                        }

                                        if (dist <= 1.0e-14)
                                        {
                                            SameEdge = true;
                                            break;
                                        }
                                    }
                                }
                            }
                        }
                    }
                }

                if (SameEdge)
                {
                    OutL.Append(E);
                    continue;
                }

                if (myDS.EdgeHasSplE(E))
                {
                    NCollection_List<Shape>.Iterator itS = new();
                    for (itS.Initialize(myDS.EdgeSplE(E)); itS.More(); itS.Next())
                    {
                        Shape newE = itS.Value();
                        newE.Orientation(Internal);
                        if (W.IsNull)
                        {
                            B.MakeWire(ref W);
                        }
                        myDS.AddOldS(newE, F);
                        B.Add(W, newE);
                    }
                }
                else
                {
                    if (W.IsNull)
                    {
                        B.MakeWire(ref W);
                    }
                    myDS.AddOldS(E, F);
                    B.Add(W, E);
                }
            }
            if (!W.IsNull)
            {
                B.Add(NF, W); // add the new wire in the new face.
            }
        }

        if (myDS.FaceHasIsoL(F))
        { // get the IsoLines on face F
            Wire W = new();
            NCollection_List<Shape>.Iterator itE = new();
            for (itE.Initialize(myDS.FaceIsoL(F)); itE.More(); itE.Next())
            {
                Edge E = ShapeCasts.Edge(itE.Value());
                E.Orientation(Internal);
                if (myDS.EdgeHasSplE(E))
                { // normally IsoLines are never split.
                    NCollection_List<Shape>.Iterator itS = new();
                    for (itS.Initialize(myDS.EdgeSplE(E)); itS.More(); itS.Next())
                    {
                        Shape newE = itS.Value();
                        newE.Orientation(Internal);
                        if (W.IsNull)
                        {
                            B.MakeWire(ref W);
                        }
                        myDS.AddOldS(newE, F);
                        B.Add(W, newE);
                    }
                }
                else
                {
                    if (W.IsNull)
                    {
                        B.MakeWire(ref W);
                    }
                    myDS.AddOldS(E, F);
                    B.Add(W, E);
                }
            }
            if (!W.IsNull)
            {
                B.Add(NF, W); // add the new wire in the new face.
            }
        }
        myDS.AddOldS(NF, F);
        // OCCT copies the adaptors' tool into the new face's entry; the port's entries share it (decision 1096)
        MST.Bind(NF, MST.ChangeFind(F));
        B.Add(S, NF); // add the face in the shell.
    }

    /// <summary>Builds the outlined shape: the shells and the free faces outlined, the free edges as they are.</summary>
    private void BuildShape(NCollection_DataMap<Shape, BRepTopAdaptor_Tool> MST)
    {
        Explorer exshell = new(), exface = new(), exedge = new();
        BrepBuilder B = new();
        Compound aCompound = new();
        B.MakeCompound(ref aCompound);
        myOutLinedShape = aCompound;
        NCollection_Map<Shape> ShapeMap = new(ShapeHasher.Instance);

        for (exshell.Init(myWorkingShape, ShapeType.Shell); exshell.More(); exshell.Next())
        { // faces in a shell (open or close)

            Shell theShell = new();
            B.MakeShell(ref theShell);
            theShell.Closed(exshell.Current().Closed());

            for (exface.Init(exshell.Current(), ShapeType.Face); exface.More(); exface.Next())
            {
                if (ShapeMap.Add(exface.Current()))
                {
                    Shape aShell = theShell;
                    ProcessFace(ShapeCasts.Face(exface.Current()), ref aShell, MST);
                }
            }
            B.Add(myOutLinedShape, theShell);
        }

        for (exface.Init(myWorkingShape, ShapeType.Face, ShapeType.Shell); exface.More(); exface.Next())
        { // faces not in a shell
            if (ShapeMap.Add(exface.Current()))
            {
                ProcessFace(ShapeCasts.Face(exface.Current()), ref myOutLinedShape, MST);
            }
        }

        for (exedge.Init(myWorkingShape, ShapeType.Edge, ShapeType.Face); exedge.More(); exedge.Next())
        { // edges not in a face
            B.Add(myOutLinedShape, exedge.Current());
        }
    }
}
