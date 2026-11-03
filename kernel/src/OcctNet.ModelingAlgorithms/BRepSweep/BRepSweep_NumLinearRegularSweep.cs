// Port of: src/ModelingAlgorithms/TKPrim/BRepSweep/BRepSweep_NumLinearRegularSweep.hxx
// Port of: src/ModelingAlgorithms/TKPrim/BRepSweep/BRepSweep_NumLinearRegularSweep.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Topology.Brep;
using Kerf.Topology;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Occt;

/// <summary>
/// The topological sweep of a generating shape along a directing indexed edge (<see cref="Sweep_NumShape"/>): the
/// result shape of each pair (generating sub-shape, directing sub-shape) is built on demand from the results of their
/// sub-shapes, with the geometry given by the derived class.
/// </summary>
/// <remarks>
/// Internal (decision 32xx-S1): the shapes it builds are unfrozen and, without copy, it writes into the generating
/// shape's sub-shapes; callers use <see cref="BRepPrimAPI_MakePrism"/> and <see cref="BRepPrimAPI_MakeRevol"/>.
/// </remarks>
internal abstract class BRepSweep_NumLinearRegularSweep
{
    protected readonly BRepSweep_Builder myBuilder;
    protected readonly Shape myGenShape;
    protected readonly Sweep_NumShape myDirWire;
    protected readonly BRepSweep_Tool myGenShapeTool;
    protected readonly Sweep_NumShapeTool myDirShapeTool;
    protected readonly NCollection_Array2<Shape> myShapes;
    protected readonly NCollection_Array2<bool> myBuiltShapes;
    protected readonly NCollection_Array2<bool> myUsedShapes;

    protected BRepSweep_NumLinearRegularSweep(BRepSweep_Builder aBuilder, in Shape aGenShape, in Sweep_NumShape aDirShape)
    {
        myBuilder = aBuilder;
        myGenShape = aGenShape;
        myDirWire = aDirShape;
        myGenShapeTool = new BRepSweep_Tool(aGenShape);
        myDirShapeTool = new Sweep_NumShapeTool(aDirShape);
        // Les Tableaux
        myShapes = new NCollection_Array2<Shape>(1, myGenShapeTool.NbShapes(), 1, myDirShapeTool.NbShapes());
        myBuiltShapes = new NCollection_Array2<bool>(1, myGenShapeTool.NbShapes(), 1, myDirShapeTool.NbShapes());
        myUsedShapes = new NCollection_Array2<bool>(1, myGenShapeTool.NbShapes(), 1, myDirShapeTool.NbShapes());
        myBuiltShapes.Init(false);
        myUsedShapes.Init(false);
    }

    /// <summary>Builds the vertex addressed by [aGenV,aDirV], with its geometric part, but without sub-shapes.</summary>
    public abstract Shape MakeEmptyVertex(in Shape aGenV, in Sweep_NumShape aDirV);

    /// <summary>Builds the edge addressed by [aGenV,aDirE], with its geometric part, but without sub-shapes.</summary>
    public abstract Shape MakeEmptyDirectingEdge(in Shape aGenV, in Sweep_NumShape aDirE);

    /// <summary>Builds the edge addressed by [aGenE,aDirV], with its geometric part, but without sub-shapes.</summary>
    public abstract Shape MakeEmptyGeneratingEdge(in Shape aGenE, in Sweep_NumShape aDirV);

    /// <summary>Sets the parameters of the new vertex on the new face (directly in a cap face).</summary>
    public abstract void SetParameters(in Shape aNewFace, ref Shape aNewVertex, in Shape aGenF,
                                       in Shape aGenV, in Sweep_NumShape aDirV);

    /// <summary>Sets the parameter of the new vertex on the new edge (the edge was built from a vertex).</summary>
    public abstract void SetDirectingParameter(in Shape aNewEdge, ref Shape aNewVertex, in Shape aGenV,
                                               in Sweep_NumShape aDirE, in Sweep_NumShape aDirV);

    /// <summary>Sets the parameter of the new vertex on the new edge (the edge was built from an edge).</summary>
    public abstract void SetGeneratingParameter(in Shape aNewEdge, ref Shape aNewVertex, in Shape aGenE,
                                                in Shape aGenV, in Sweep_NumShape aDirV);

    /// <summary>Builds the face addressed by [aGenS,aDirS], with its geometric part, but without sub-shapes.</summary>
    public abstract Shape MakeEmptyFace(in Shape aGenS, in Sweep_NumShape aDirS);

    /// <summary>Sets the pcurve of the new edge on the new face (a cap face).</summary>
    public abstract void SetPCurve(in Shape aNewFace, ref Shape aNewEdge, in Shape aGenF,
                                   in Shape aGenE, in Sweep_NumShape aDirV, Orientation orien);

    /// <summary>Sets the pcurve of the new edge (built from an edge) on the new face.</summary>
    public abstract void SetGeneratingPCurve(in Shape aNewFace, ref Shape aNewEdge, in Shape aGenE,
                                             in Sweep_NumShape aDirE, in Sweep_NumShape aDirV, Orientation orien);

    /// <summary>Sets the pcurve of the new edge (built from a vertex) on the new face.</summary>
    public abstract void SetDirectingPCurve(in Shape aNewFace, ref Shape aNewEdge, in Shape aGenE,
                                            in Shape aGenV, in Sweep_NumShape aDirE, Orientation orien);

    /// <summary>The orientation of the shell in the solid generated by the face and the directing edge.</summary>
    public abstract Orientation DirectSolid(in Shape aGenS, in Sweep_NumShape aDirS);

    /// <summary>True if aNewSubShape (addressed by [aSubGenS,aDirS]) is to be added to aNewShape ([aGenS,aDirS]).</summary>
    public abstract bool GGDShapeIsToAdd(in Shape aNewShape, in Shape aNewSubShape, in Shape aGenS,
                                         in Shape aSubGenS, in Sweep_NumShape aDirS);

    /// <summary>True if aNewSubShape (addressed by [aGenS,aSubDirS]) is to be added to aNewShape ([aGenS,aDirS]).</summary>
    public abstract bool GDDShapeIsToAdd(in Shape aNewShape, in Shape aNewSubShape, in Shape aGenS,
                                         in Sweep_NumShape aDirS, in Sweep_NumShape aSubDirS);

    /// <summary>In some particular cases the topology of a generated face must be composed of independent closed wires.</summary>
    public abstract bool SeparatedWires(in Shape aNewShape, in Shape aNewSubShape, in Shape aGenS,
                                        in Shape aSubGenS, in Sweep_NumShape aDirS);

    /// <summary>In some particular cases the topology of a generated shell must be split in several shells.</summary>
    public virtual Shape SplitShell(in Shape aNewShape)
    {
        Shape comp = new();
        myBuilder.MakeCompound(ref comp);
        myBuilder.Add(comp, aNewShape);
        return comp;
    }

    /// <summary>Called to set the continuity of the new edges.</summary>
    public abstract void SetContinuity(in Shape aGenS, in Sweep_NumShape aDirS);

    /// <summary>True if the shape addressed by [aGenS,aDirS] exists.</summary>
    public abstract bool HasShape(in Shape aGenS, in Sweep_NumShape aDirS);

    /// <summary>True if aGenS can't be transformed by the sweep.</summary>
    public abstract bool IsInvariant(in Shape aGenS);

    /// <summary>The resulting shape indexed by aDirS and aGenS.</summary>
    public Shape Shape(in Shape aGenS, in Sweep_NumShape aDirS)
    {
        int iGenS = myGenShapeTool.Index(aGenS);
        int iDirS = myDirShapeTool.Index(aDirS);
        if (!myBuiltShapes[iGenS, iDirS])
        {
            Shape newShape;
            Shape bGenS, cGenS, subGenS, subsubGenS;
            Sweep_NumShape bDirS, subDirS;
            var It = new BRepSweep_Iterator();
            var Kt = new Sweep_NumShapeIterator();
            var Lt = new BRepSweep_Iterator();
            Orientation Or, Pr;
            if (myDirShapeTool.Type(aDirS) == ShapeType.Vertex)
            {
                // Ici on construit les "planchers" du Shape.
                ShapeType aGenSType = myGenShapeTool.Type(aGenS);
                switch (aGenSType)
                {
                    case ShapeType.Vertex:
                        myShapes[iGenS, iDirS] = MakeEmptyVertex(aGenS, aDirS);
                        break;
                    case ShapeType.Edge:
                        myShapes[iGenS, iDirS] = MakeEmptyGeneratingEdge(aGenS, aDirS);
                        break;
                    case ShapeType.Wire:
                        myBuilder.MakeWire(ref myShapes[iGenS, iDirS]);
                        break;
                    case ShapeType.Face:
                        myShapes[iGenS, iDirS] = MakeEmptyFace(aGenS, aDirS);
                        break;
                    case ShapeType.Shell:
                        myBuilder.MakeShell(ref myShapes[iGenS, iDirS]);
                        break;
                    case ShapeType.Solid:
                        throw new NoSuchObjectException("Solids are not Processed");
                    case ShapeType.CompSolid:
                        throw new NoSuchObjectException("Solids are not Processed");
                    case ShapeType.Compound:
                        myBuilder.MakeCompound(ref myShapes[iGenS, iDirS]);
                        break;
                    default:
                        throw new NoSuchObjectException("Unknown Shape");
                }
                bGenS = aGenS;
                myGenShapeTool.SetOrientation(ref bGenS, Forward);
                for (It.Init(bGenS); It.More(); It.Next())
                {
                    subGenS = It.Value();
                    Or = It.Orientation();
                    if (HasShape(subGenS, aDirS))
                    {
                        newShape = Shape(subGenS, aDirS);
                        int iNewGenS = myGenShapeTool.Index(subGenS);
                        int iNewDirS = iDirS;
                        if (GGDShapeIsToAdd(myShapes[iGenS, iDirS], newShape, aGenS, subGenS, aDirS))
                        {
                            // Les "planchers" doivent etre construits par les
                            // fonctions de construcion geometrique identiquement
                            // au shape generateur.
                            // On leur recolle juste une orientation pour etre bien
                            // sur.
                            myBuilder.Add(myShapes[iGenS, iDirS], newShape, Or);
                            myUsedShapes[iNewGenS, iNewDirS] = true;
                            ShapeType subGenSType = myGenShapeTool.Type(subGenS);
                            if (aGenSType == ShapeType.Face)
                            {
                                if (subGenSType == ShapeType.Vertex)
                                {
                                    SetParameters(myShapes[iGenS, iDirS], ref newShape, aGenS, subGenS, aDirS);
                                }
                                else if (subGenSType == ShapeType.Edge)
                                {
                                    SetPCurve(myShapes[iGenS, iDirS], ref newShape, aGenS, subGenS, aDirS, Or);
                                }
                                else if (subGenSType == ShapeType.Wire)
                                {
                                    var Jt = new BRepSweep_Iterator();
                                    cGenS = subGenS;
                                    myGenShapeTool.SetOrientation(ref cGenS, Forward);
                                    for (Jt.Init(cGenS); Jt.More(); Jt.Next())
                                    {
                                        subsubGenS = Jt.Value();
                                        Pr = Jt.Orientation();
                                        if (HasShape(subsubGenS, aDirS))
                                        {
                                            Shape newsubEdge = Shape(subsubGenS, aDirS);
                                            SetPCurve(myShapes[iGenS, iDirS], ref newsubEdge, aGenS, subsubGenS, aDirS, Pr);
                                        }
                                    }
                                }
                            }
                            else if (aGenSType == ShapeType.Edge)
                            {
                                SetGeneratingParameter(myShapes[iGenS, iDirS], ref newShape, bGenS, subGenS, aDirS);
                            }
                        }
                    }
                }
            }
            else if (myDirShapeTool.Type(aDirS) == ShapeType.Edge)
            {
                // Ici on construit les murs du Shape.
                ShapeType aGenSType = myGenShapeTool.Type(aGenS);
                Shape newWire = new(), newShell = new();
                var WireSeq = new NCollection_Sequence<Shape>();
                bool sepwires = false;
                switch (aGenSType)
                {
                    case ShapeType.Vertex:
                        myShapes[iGenS, iDirS] = MakeEmptyDirectingEdge(aGenS, aDirS);
                        break;
                    case ShapeType.Edge:
                        // On cree un wire intermediaire qui contient tous les edges
                        // du montant (face) du Shape pour le cas standard, et une
                        // sequence de wires pour les cas merdiques necessitant des
                        // wires independants.
                        myBuilder.MakeWire(ref newWire);
                        myShapes[iGenS, iDirS] = MakeEmptyFace(aGenS, aDirS);
                        break;
                    case ShapeType.Wire:
                        myBuilder.MakeShell(ref myShapes[iGenS, iDirS]);
                        break;
                    case ShapeType.Face:
                        // On cree un shell intermediaire dans lequel on jette toutes
                        // les faces en direct, pour eviter les empilages compliques
                        // de shells et sous shells dans la structure du solide.
                        myBuilder.MakeShell(ref newShell);
                        myBuilder.MakeSolid(ref myShapes[iGenS, iDirS]);
                        break;
                    case ShapeType.Shell:
                        myBuilder.MakeCompSolid(ref myShapes[iGenS, iDirS]);
                        break;
                    case ShapeType.Solid:
                        throw new NoSuchObjectException("Solids are not Processed");
                    case ShapeType.CompSolid:
                        throw new NoSuchObjectException("Solids are not Processed");
                    case ShapeType.Compound:
                        myBuilder.MakeCompound(ref myShapes[iGenS, iDirS]);
                        break;
                    default:
                        throw new NoSuchObjectException("Unknown Shape");
                }
                bGenS = aGenS;
                myGenShapeTool.SetOrientation(ref bGenS, Forward);
                for (It.Init(bGenS); It.More(); It.Next())
                {
                    subGenS = It.Value();
                    if (HasShape(subGenS, aDirS))
                    {
                        newShape = Shape(subGenS, aDirS);
                        int iNewGenS = myGenShapeTool.Index(subGenS);
                        int iNewDirS = iDirS;
                        if (GGDShapeIsToAdd(myShapes[iGenS, iDirS], newShape, aGenS, subGenS, aDirS))
                        {
                            ShapeType subGenSType = myGenShapeTool.Type(subGenS);
                            if (aGenSType == ShapeType.Edge)
                            {
                                Or = It.Orientation();
                                if (SeparatedWires(myShapes[iGenS, iDirS], newShape, aGenS, subGenS, aDirS))
                                {
                                    sepwires = true;
                                    Shape wi = new();
                                    myBuilder.MakeWire(ref wi);
                                    myBuilder.Add(wi, newShape, Or);
                                    myUsedShapes[iNewGenS, iNewDirS] = true;
                                    wi.Closed(Tool.IsClosed(wi));
                                    WireSeq.Append(wi);
                                }
                                else
                                {
                                    myBuilder.Add(newWire, newShape, Or);
                                    myUsedShapes[iNewGenS, iNewDirS] = true;
                                }
                                SetDirectingPCurve(myShapes[iGenS, iDirS], ref newShape, bGenS, subGenS, aDirS, Or);
                            }
                            else if (aGenSType == ShapeType.Wire)
                            {
                                Or = It.Orientation();
                                myBuilder.Add(myShapes[iGenS, iDirS], newShape, Or);
                                myUsedShapes[iNewGenS, iNewDirS] = true;
                            }
                            else if (aGenSType == ShapeType.Face)
                            {
                                Or = It.Orientation();
                                if (subGenSType == ShapeType.Wire)
                                {
                                    for (Lt.Init(newShape); Lt.More(); Lt.Next())
                                    {
                                        myBuilder.Add(newShell, Lt.Value(), global::Kerf.Engine.Topology.TopAbs.Compose(Lt.Orientation(), Or));
                                    }
                                }
                                else if (subGenSType == ShapeType.Edge)
                                {
                                    myBuilder.Add(newShell, newShape, Or);
                                    myUsedShapes[iNewGenS, iNewDirS] = true;
                                }
                            }
                            else if (aGenSType == ShapeType.Shell)
                            {
                                Or = Forward;
                                myBuilder.Add(myShapes[iGenS, iDirS], newShape, Or);
                                myUsedShapes[iNewGenS, iNewDirS] = true;
                            }
                            else if (aGenSType == ShapeType.Compound)
                            {
                                Or = Forward;
                                myBuilder.Add(myShapes[iGenS, iDirS], newShape, Or);
                                myUsedShapes[iNewGenS, iNewDirS] = true;
                            }
                            else
                            {
                                Or = It.Orientation();
                                myBuilder.Add(myShapes[iGenS, iDirS], newShape, Or);
                                myUsedShapes[iNewGenS, iNewDirS] = true;
                            }
                        }
                    }
                }
                bDirS = aDirS;
                for (Kt.Init(bDirS); Kt.More(); Kt.Next())
                {
                    subDirS = Kt.Value();
                    if (HasShape(aGenS, subDirS))
                    {
                        newShape = Shape(aGenS, subDirS);
                        int iNewGenS = iGenS;
                        int iNewDirS = myDirShapeTool.Index(subDirS);
                        if (GDDShapeIsToAdd(myShapes[iGenS, iDirS], newShape, aGenS, aDirS, subDirS))
                        {
                            if (aGenSType == ShapeType.Edge)
                            {
                                Or = global::Kerf.Engine.Topology.TopAbs.Reverse(Kt.Orientation());
                                myBuilder.Add(newWire, newShape, Or);
                                myUsedShapes[iNewGenS, iNewDirS] = true;
                                SetGeneratingPCurve(myShapes[iGenS, iDirS], ref newShape, aGenS, aDirS, subDirS, Or);
                            }
                            else if (aGenSType == ShapeType.Vertex)
                            {
                                Or = Kt.Orientation();
                                myBuilder.Add(myShapes[iGenS, iDirS], newShape, Or);
                                myUsedShapes[iNewGenS, iNewDirS] = true;
                                SetDirectingParameter(myShapes[iGenS, iDirS], ref newShape, aGenS, aDirS, subDirS);
                            }
                            else if (aGenSType == ShapeType.Face)
                            {
                                Or = Kt.Orientation();
                                myBuilder.Add(newShell, newShape, Or);
                                myUsedShapes[iNewGenS, iNewDirS] = true;
                            }
                        }
                    }
                }
                if (aGenSType == ShapeType.Edge)
                {
                    if (sepwires)
                    {
                        for (int ij = 1; ij <= WireSeq.Length(); ij++)
                        {
                            myBuilder.Add(myShapes[iGenS, iDirS], WireSeq.Value(ij));
                        }
                    }
                    else
                    {
                        newWire.Closed(Tool.IsClosed(newWire));
                        myBuilder.Add(myShapes[iGenS, iDirS], newWire);
                    }
                    myBuiltShapes[iGenS, iDirS] = true;
                    SetContinuity(aGenS, aDirS);
                }
                if (aGenSType == ShapeType.Wire)
                {
                    SetContinuity(aGenS, aDirS);
                }
                if (aGenSType == ShapeType.Face)
                {
                    newShell.Closed(Tool.IsClosed(newShell));
                    Shape temp = SplitShell(newShell);
                    Orientation ShellOri = DirectSolid(aGenS, aDirS);
                    Lt.Init(temp);
                    if (Lt.More())
                    {
                        Lt.Next();
                    }
                    if (Lt.More())
                    {
                        for (Lt.Init(temp); Lt.More(); Lt.Next())
                        {
                            myBuilder.Add(myShapes[iGenS, iDirS], Lt.Value(), ShellOri);
                        }
                    }
                    else
                    {
                        myBuilder.Add(myShapes[iGenS, iDirS], newShell, ShellOri);
                    }
                }
            }
            else if (myDirShapeTool.Type(aDirS) == ShapeType.Wire)
            {
                ShapeType aGenSType = myGenShapeTool.Type(aGenS);
                switch (aGenSType)
                {
                    case ShapeType.Vertex:
                        myBuilder.MakeWire(ref myShapes[iGenS, iDirS]);
                        break;
                    case ShapeType.Edge:
                        myBuilder.MakeShell(ref myShapes[iGenS, iDirS]);
                        break;
                    case ShapeType.Wire:
                        myBuilder.MakeShell(ref myShapes[iGenS, iDirS]);
                        break;
                    case ShapeType.Face:
                        myBuilder.MakeCompSolid(ref myShapes[iGenS, iDirS]);
                        break;
                    case ShapeType.Shell:
                        myBuilder.MakeCompSolid(ref myShapes[iGenS, iDirS]);
                        break;
                    case ShapeType.Solid:
                        throw new NoSuchObjectException("Solids are not Processed");
                    case ShapeType.CompSolid:
                        throw new NoSuchObjectException("Solids are not Processed");
                    case ShapeType.Compound:
                        myBuilder.MakeCompound(ref myShapes[iGenS, iDirS]);
                        break;
                    default:
                        throw new NoSuchObjectException("Unknown Shape");
                }
                bDirS = aDirS;
                for (Kt.Init(aDirS); Kt.More(); Kt.Next())
                {
                    subDirS = Kt.Value();
                    if (HasShape(aGenS, subDirS))
                    {
                        int iNewGenS = iGenS;
                        int iNewDirS = myDirShapeTool.Index(subDirS);
                        Or = Kt.Orientation();
                        newShape = Shape(aGenS, subDirS);
                        myBuilder.Add(myShapes[iGenS, iDirS], newShape, Or);
                        myUsedShapes[iNewGenS, iNewDirS] = true;
                    }
                }
            }
            myBuiltShapes[iGenS, iDirS] = true;
        }
        // Change the "Closed" flag only for Wires and Shells
        if (myShapes[iGenS, iDirS].ShapeType() == ShapeType.Wire || myShapes[iGenS, iDirS].ShapeType() == ShapeType.Shell)
        {
            // OCCT.NET: frozen. OCCT sets the flag again on each call; a result already published (frozen), or a
            // generating shape's own wire, is written only if the flag changes.
            bool isClosed = Tool.IsClosed(myShapes[iGenS, iDirS]);
            if (myShapes[iGenS, iDirS].Closed() != isClosed)
            {
                myShapes[iGenS, iDirS].Closed(isClosed);
            }
        }
        return myShapes[iGenS, iDirS];
    }

    /// <summary>The resulting shape generated with aGenS (null if there is none).</summary>
    public Shape Shape(in Shape aGenS)
    {
        if (myGenShapeTool.Index(aGenS) != 0 && HasShape(aGenS, myDirWire))
        {
            return Shape(aGenS, myDirWire);
        }
        else
        {
            Shape bidon = new();
            return bidon;
        }
    }

    /// <summary>True if the initial shape aGenS is used in the result shape.</summary>
    public bool IsUsed(in Shape aGenS)
    {
        int iGenS = myGenShapeTool.Index(aGenS);
        if (iGenS == 0)
        {
            return false;
        }
        int j;
        bool isBuilt = false;
        bool isUsed = false;
        for (j = 2; j <= myBuiltShapes.UpperCol(); ++j)
        {
            isBuilt = isBuilt || myBuiltShapes[iGenS, j];
            isUsed = isUsed || myUsedShapes[iGenS, j];
        }
        if (isUsed)
        {
            if (aGenS.ShapeType() == ShapeType.Vertex && IsInvariant(aGenS))
            {
                if (myUsedShapes[iGenS, 1] || !Closed())
                {
                    return isUsed;
                }
                else
                {
                    return false;
                }
            }
            else
            {
                return isUsed;
            }
        }
        //
        if (isBuilt) //&& !IsUsed
        {
            if (!HasShape(aGenS, myDirWire) && !Closed())
            {
                return true;
            }
            else if (aGenS.ShapeType() == ShapeType.Vertex && !Closed())
            {
                if (!myBuiltShapes[iGenS, 1])
                {
                    return true;
                }
            }
        }
        return isUsed;
    }

    /// <summary>True if the shape generated from theS (a sub-shape of the generating shape) is used in the result.</summary>
    public bool GenIsUsed(in Shape aGenS)
    {
        int iGenS = myGenShapeTool.Index(aGenS);
        if (iGenS == 0)
        {
            return false;
        }
        if (iGenS == 1)
        {
            return myBuiltShapes[iGenS, 1];
        }
        else
        {
            return myBuiltShapes[iGenS, 1] && myUsedShapes[iGenS, 1];
        }
    }

    /// <summary>The resulting shape (null if there is none).</summary>
    public Shape Shape()
    {
        if (HasShape(myGenShape, myDirWire))
        {
            return Shape(myGenShape, myDirWire);
        }
        else
        {
            Shape bidon = new();
            return bidon;
        }
    }

    /// <summary>The resulting shape indexed by the first vertex of myDirWire and myGenShape.</summary>
    public Shape FirstShape()
    {
        Shape result = new();
        if (myDirShapeTool.HasFirstVertex())
        {
            if (HasShape(myGenShape, myDirShapeTool.FirstVertex()))
            {
                result = Shape(myGenShape, myDirShapeTool.FirstVertex());
            }
        }
        return result;
    }

    /// <summary>The resulting shape indexed by the last vertex of myDirWire and myGenShape.</summary>
    public Shape LastShape()
    {
        Shape result = new();
        if (myDirShapeTool.HasLastVertex())
        {
            if (HasShape(myGenShape, myDirShapeTool.LastVertex()))
            {
                result = Shape(myGenShape, myDirShapeTool.LastVertex());
            }
        }
        return result;
    }

    /// <summary>The resulting shape indexed by the first vertex of myDirWire and aGenS.</summary>
    public Shape FirstShape(in Shape aGenS)
    {
        Shape result = new();
        if (myDirShapeTool.HasFirstVertex())
        {
            if (HasShape(aGenS, myDirShapeTool.FirstVertex()))
            {
                result = Shape(aGenS, myDirShapeTool.FirstVertex());
            }
        }
        return result;
    }

    /// <summary>The resulting shape indexed by the last vertex of myDirWire and aGenS.</summary>
    public Shape LastShape(in Shape aGenS)
    {
        Shape result = new();
        if (myDirShapeTool.HasLastVertex())
        {
            if (HasShape(aGenS, myDirShapeTool.LastVertex()))
            {
                result = Shape(aGenS, myDirShapeTool.LastVertex());
            }
        }
        return result;
    }

    /// <summary>True if the directing wire is closed.</summary>
    public bool Closed() => myDirWire.Closed();
}
