// Port of: src/ModelingAlgorithms/TKHLR/HLRTopoBRep/HLRTopoBRep_OutLiner.hxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRTopoBRep/HLRTopoBRep_OutLiner.lxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRTopoBRep/HLRTopoBRep_OutLiner.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Geometry;
using static Kerf.Geometry.CurveKind;
using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;

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
    private TopoDS_Shape myOriginalShape = new();
    private TopoDS_Shape myOutLinedShape = new();
    private TopoDS_Shape myWorkingShape = new();
    private Dictionary<TopoDS_TShape, TopoDS_TShape>? myCopies; // the original's TShapes to the working shape's (7Q2)
    private readonly HLRTopoBRep_Data myDS = new();

    public HLRTopoBRep_OutLiner()
    {
    }

    public HLRTopoBRep_OutLiner(in TopoDS_Shape OriSh)
    {
        myOriginalShape = OriSh;
    }

    public HLRTopoBRep_OutLiner(in TopoDS_Shape OriS, in TopoDS_Shape OutS)
    {
        myOriginalShape = OriS;
        myOutLinedShape = OutS;
    }

    public void OriginalShape(in TopoDS_Shape OriS) => myOriginalShape = OriS;

    /// <summary>The caller's original shape.</summary>
    public TopoDS_Shape OriginalShape() => myOriginalShape;

    public void OutLinedShape(in TopoDS_Shape OutS) => myOutLinedShape = OutS;

    /// <summary>The outlined shape (frozen).</summary>
    public TopoDS_Shape OutLinedShape()
    {
        myOutLinedShape.TShape()?.FreezeGraph();
        return myOutLinedShape;
    }

    /// <summary><see cref="OutLinedShape()"/> without freezing, for ported code.</summary>
    internal TopoDS_Shape InternalOutLinedShape() => myOutLinedShape;

    /// <summary>
    /// The shape the outlining works on: the original shape, or its copy when it held frozen TShapes (decision 1095).
    /// The original shape before <see cref="Fill"/>.
    /// </summary>
    internal TopoDS_Shape InternalWorkingShape() => myWorkingShape.IsNull() ? myOriginalShape : myWorkingShape;

    /// <summary>
    /// The image of a sub-shape of the original shape in the working shape (the same place and orientation, the
    /// copy's TShape); the shape itself when the original wasn't copied or the shape isn't one of its sub-shapes
    /// (unit 7Q2: HLRBRep_HLRToShape finds a caller's sub-shapes in the data structure through it).
    /// </summary>
    internal TopoDS_Shape InternalWorkingImage(in TopoDS_Shape S)
    {
        if (myCopies is not null && !S.IsNull() && myCopies.TryGetValue(S.TShape()!, out TopoDS_TShape? anImage))
        {
            TopoDS_Shape aShape = S;
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
    public void Fill(in HLRAlgo_Projector P, NCollection_DataMap<TopoDS_Shape, BRepTopAdaptor_Tool> MST, int nbIso)
    {
        if (!myOriginalShape.IsNull())
        {
            if (myOutLinedShape.IsNull())
            {
                // copy at the entry (decision 1095)
                myCopies = null;
                myWorkingShape = BRepTools.HoldsFrozen(myOriginalShape)
                                     ? BRepTools.ExactCopyForWrite(myOriginalShape, out _, out myCopies)
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
    private void ProcessFace(in TopoDS_Face F, ref TopoDS_Shape S,
                             NCollection_DataMap<TopoDS_Shape, BRepTopAdaptor_Tool> MST)
    {
        BRep_Builder B = new();
        TopExp_Explorer exE = new(), exW = new();

        NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> aVEMap =
            new(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapesAndAncestors(F, TopAbs_VERTEX, TopAbs_EDGE, aVEMap);

        TopoDS_Shape NF; // = F;
        NF = F.EmptyCopied();

        for (exW.Init(F, TopAbs_WIRE); exW.More(); exW.Next())
        {
            TopoDS_Wire W = new();
            B.MakeWire(ref W);

            for (exE.Init(exW.Current(), TopAbs_EDGE); exE.More(); exE.Next())
            {
                TopoDS_Edge E = TopoDS.Edge(exE.Current());
                if (myDS.EdgeHasSplE(E))
                {
                    NCollection_List<TopoDS_Shape>.Iterator itS = new();
                    for (itS.Initialize(myDS.EdgeSplE(E)); itS.More(); itS.Next())
                    {
                        TopoDS_Edge newE = TopoDS.Edge(itS.Value());
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
        NCollection_List<TopoDS_Shape> OutL = myDS.AddOutL(F);

        if (myDS.FaceHasIntL(F))
        { // get the InternalOutLines on face F
            TopoDS_Wire W = new();
            NCollection_List<TopoDS_Shape>.Iterator itE = new();
            for (itE.Initialize(myDS.FaceIntL(F)); itE.More(); itE.Next())
            {
                TopoDS_Edge E = TopoDS.Edge(itE.Value());
                E.Orientation(TopAbs_INTERNAL);

                // Check, if outline edge coincides with face boundary
                BRepAdaptor_Curve C = new(E);
                double par = 0.34 * C.FirstParameter() + 0.66 * C.LastParameter();
                Point3D P = C.Value(par);
                TopoDS_Vertex V1, V2, aV1, aV2;
                TopExp.Vertices(E, out V1, out V2);
                bool SameEdge = false;
                if (!V1.IsNull() && aVEMap.Contains(V1))
                {
                    NCollection_List<TopoDS_Shape> aEList = aVEMap.FindFromKey(V1);
                    NCollection_List<TopoDS_Shape>.Iterator it = new(aEList);
                    for (; it.More(); it.Next())
                    {
                        TopoDS_Edge aE = TopoDS.Edge(it.Value());
                        TopExp.Vertices(aE, out aV1, out aV2);

                        if ((V1.IsSame(aV1) && V2.IsSame(aV2)) || (V1.IsSame(aV2) && V2.IsSame(aV1)))
                        {
                            BRepAdaptor_Curve aC = new(aE);
                            if ((C.GetType() == Line) && (aC.GetType() == Line))
                            {
                                SameEdge = true;
                                break;
                            }
                            else
                            {
                                Extrema_ExtPC anExt = new(P, aC);
                                if (anExt.IsDone())
                                {
                                    int aNe = anExt.NbExt();
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
                    NCollection_List<TopoDS_Shape>.Iterator itS = new();
                    for (itS.Initialize(myDS.EdgeSplE(E)); itS.More(); itS.Next())
                    {
                        TopoDS_Shape newE = itS.Value();
                        newE.Orientation(TopAbs_INTERNAL);
                        if (W.IsNull())
                        {
                            B.MakeWire(ref W);
                        }
                        myDS.AddOldS(newE, F);
                        B.Add(W, newE);
                    }
                }
                else
                {
                    if (W.IsNull())
                    {
                        B.MakeWire(ref W);
                    }
                    myDS.AddOldS(E, F);
                    B.Add(W, E);
                }
            }
            if (!W.IsNull())
            {
                B.Add(NF, W); // add the new wire in the new face.
            }
        }

        if (myDS.FaceHasIsoL(F))
        { // get the IsoLines on face F
            TopoDS_Wire W = new();
            NCollection_List<TopoDS_Shape>.Iterator itE = new();
            for (itE.Initialize(myDS.FaceIsoL(F)); itE.More(); itE.Next())
            {
                TopoDS_Edge E = TopoDS.Edge(itE.Value());
                E.Orientation(TopAbs_INTERNAL);
                if (myDS.EdgeHasSplE(E))
                { // normally IsoLines are never split.
                    NCollection_List<TopoDS_Shape>.Iterator itS = new();
                    for (itS.Initialize(myDS.EdgeSplE(E)); itS.More(); itS.Next())
                    {
                        TopoDS_Shape newE = itS.Value();
                        newE.Orientation(TopAbs_INTERNAL);
                        if (W.IsNull())
                        {
                            B.MakeWire(ref W);
                        }
                        myDS.AddOldS(newE, F);
                        B.Add(W, newE);
                    }
                }
                else
                {
                    if (W.IsNull())
                    {
                        B.MakeWire(ref W);
                    }
                    myDS.AddOldS(E, F);
                    B.Add(W, E);
                }
            }
            if (!W.IsNull())
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
    private void BuildShape(NCollection_DataMap<TopoDS_Shape, BRepTopAdaptor_Tool> MST)
    {
        TopExp_Explorer exshell = new(), exface = new(), exedge = new();
        BRep_Builder B = new();
        TopoDS_Compound aCompound = new();
        B.MakeCompound(ref aCompound);
        myOutLinedShape = aCompound;
        NCollection_Map<TopoDS_Shape> ShapeMap = new(TopTools_ShapeMapHasher.Instance);

        for (exshell.Init(myWorkingShape, TopAbs_SHELL); exshell.More(); exshell.Next())
        { // faces in a shell (open or close)

            TopoDS_Shell theShell = new();
            B.MakeShell(ref theShell);
            theShell.Closed(exshell.Current().Closed());

            for (exface.Init(exshell.Current(), TopAbs_FACE); exface.More(); exface.Next())
            {
                if (ShapeMap.Add(exface.Current()))
                {
                    TopoDS_Shape aShell = theShell;
                    ProcessFace(TopoDS.Face(exface.Current()), ref aShell, MST);
                }
            }
            B.Add(myOutLinedShape, theShell);
        }

        for (exface.Init(myWorkingShape, TopAbs_FACE, TopAbs_SHELL); exface.More(); exface.Next())
        { // faces not in a shell
            if (ShapeMap.Add(exface.Current()))
            {
                ProcessFace(TopoDS.Face(exface.Current()), ref myOutLinedShape, MST);
            }
        }

        for (exedge.Init(myWorkingShape, TopAbs_EDGE, TopAbs_FACE); exedge.More(); exedge.Next())
        { // edges not in a face
            B.Add(myOutLinedShape, exedge.Current());
        }
    }
}
