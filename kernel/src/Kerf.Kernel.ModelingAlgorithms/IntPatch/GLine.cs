// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_GLine.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_GLine.lxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_GLine.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Intersection.Walking;
using Kerf.Geometry;
using Kerf.Engine;
using static Kerf.Engine.Intersection.Patches.PatchLineType;

namespace Kerf.Engine.Intersection.Patches;

/// <summary>
/// An intersection line that is a conic (a line, circle, ellipse, parabola or hyperbola), with its vertices.
/// </summary>
/// <remarks>
/// A working object (decision 609). The conic is kept as OCCT keeps it: its position (a gp_Ax2) and up to two
/// parameters (radius, radii or focal length); a line's position is the plane through it normal to it.
/// <see cref="Vertex"/>, <see cref="FirstPoint"/> and <see cref="LastPoint"/> return copies (OCCT's const
/// references). OCCT's <c>Replace</c> appends the point (its index is ignored).
/// </remarks>
internal class GLine : PatchLine
{
    private Frame3D _pos;
    private double _par1;
    private double _par2;
    private bool _fipt;
    private bool _lapt;
    private int _indf;
    private int _indl;
    private readonly NCollection_Sequence<PatchPoint> _svtx = new();

    /// <summary>Creates a line as intersection line when the transitions are in or out.</summary>
    public GLine(in Line3D l, bool tang, WalkingTypeTrans trans1, WalkingTypeTrans trans2)
        : base(tang, trans1, trans2)
    {
        _par1 = 0.0;
        _par2 = 0.0;
        _typ = PatchLineType.Lin;
        _pos = new Plane3D(l.Location, l.Direction).Position.Frame;
    }

    /// <summary>Creates a line as intersection line when the transitions are touch.</summary>
    public GLine(in Line3D l, bool tang, WalkingSituation situ1, WalkingSituation situ2)
        : base(tang, situ1, situ2)
    {
        _par1 = 0.0;
        _par2 = 0.0;
        _typ = PatchLineType.Lin;
        _pos = new Plane3D(l.Location, l.Direction).Position.Frame;
    }

    /// <summary>Creates a line as intersection line when the transitions are undecided.</summary>
    public GLine(in Line3D l, bool tang)
        : base(tang)
    {
        _par1 = 0.0;
        _par2 = 0.0;
        _typ = PatchLineType.Lin;
        _pos = new Plane3D(l.Location, l.Direction).Position.Frame;
    }

    /// <summary>Creates a circle as intersection line when the transitions are in or out.</summary>
    public GLine(in Circle3D c, bool tang, WalkingTypeTrans trans1, WalkingTypeTrans trans2)
        : base(tang, trans1, trans2)
    {
        _pos = c.Position;
        _par1 = c.Radius;
        _par2 = 0.0;
        _typ = PatchLineType.Circle;
    }

    /// <summary>Creates a circle as intersection line when the transitions are touch.</summary>
    public GLine(in Circle3D c, bool tang, WalkingSituation situ1, WalkingSituation situ2)
        : base(tang, situ1, situ2)
    {
        _pos = c.Position;
        _par1 = c.Radius;
        _par2 = 0.0;
        _typ = PatchLineType.Circle;
    }

    /// <summary>Creates a circle as intersection line when the transitions are undecided.</summary>
    public GLine(in Circle3D c, bool tang)
        : base(tang)
    {
        _pos = c.Position;
        _par1 = c.Radius;
        _par2 = 0.0;
        _typ = PatchLineType.Circle;
    }

    /// <summary>Creates an ellipse as intersection line when the transitions are in or out.</summary>
    public GLine(in Ellipse3D e, bool tang, WalkingTypeTrans trans1, WalkingTypeTrans trans2)
        : base(tang, trans1, trans2)
    {
        _pos = e.Position;
        _par1 = e.MajorRadius;
        _par2 = e.MinorRadius;
        _typ = PatchLineType.Ellipse;
    }

    /// <summary>Creates an ellipse as intersection line when the transitions are touch.</summary>
    public GLine(in Ellipse3D e, bool tang, WalkingSituation situ1, WalkingSituation situ2)
        : base(tang, situ1, situ2)
    {
        _pos = e.Position;
        _par1 = e.MajorRadius;
        _par2 = e.MinorRadius;
        _typ = PatchLineType.Ellipse;
    }

    /// <summary>Creates an ellipse as intersection line when the transitions are undecided.</summary>
    public GLine(in Ellipse3D e, bool tang)
        : base(tang)
    {
        _pos = e.Position;
        _par1 = e.MajorRadius;
        _par2 = e.MinorRadius;
        _typ = PatchLineType.Ellipse;
    }

    /// <summary>Creates a parabola as intersection line when the transitions are in or out.</summary>
    public GLine(in Parabola3D p, bool tang, WalkingTypeTrans trans1, WalkingTypeTrans trans2)
        : base(tang, trans1, trans2)
    {
        _pos = p.Position;
        _par1 = p.Focal;
        _par2 = 0.0;
        _typ = PatchLineType.Parabola;
    }

    /// <summary>Creates a parabola as intersection line when the transitions are touch.</summary>
    public GLine(in Parabola3D p, bool tang, WalkingSituation situ1, WalkingSituation situ2)
        : base(tang, situ1, situ2)
    {
        _pos = p.Position;
        _par1 = p.Focal;
        _par2 = 0.0;
        _typ = PatchLineType.Parabola;
    }

    /// <summary>Creates a parabola as intersection line when the transitions are undecided.</summary>
    public GLine(in Parabola3D p, bool tang)
        : base(tang)
    {
        _pos = p.Position;
        _par1 = p.Focal;
        _par2 = 0.0;
        _typ = PatchLineType.Parabola;
    }

    /// <summary>Creates a hyperbola as intersection line when the transitions are in or out.</summary>
    public GLine(in Hyperbola3D h, bool tang, WalkingTypeTrans trans1, WalkingTypeTrans trans2)
        : base(tang, trans1, trans2)
    {
        _pos = h.Position;
        _par1 = h.MajorRadius;
        _par2 = h.MinorRadius;
        _typ = PatchLineType.Hyperbola;
    }

    /// <summary>Creates a hyperbola as intersection line when the transitions are touch.</summary>
    public GLine(in Hyperbola3D h, bool tang, WalkingSituation situ1, WalkingSituation situ2)
        : base(tang, situ1, situ2)
    {
        _pos = h.Position;
        _par1 = h.MajorRadius;
        _par2 = h.MinorRadius;
        _typ = PatchLineType.Hyperbola;
    }

    /// <summary>Creates a hyperbola as intersection line when the transitions are undecided.</summary>
    public GLine(in Hyperbola3D h, bool tang)
        : base(tang)
    {
        _pos = h.Position;
        _par1 = h.MajorRadius;
        _par2 = h.MinorRadius;
        _typ = PatchLineType.Hyperbola;
    }

    /// <summary>Adds a vertex; on a closed conic whose first and last points are known, brought into their range.</summary>
    public void AddVertex(in PatchPoint pnt)
    {
        //-- On detecte le cas de 2 points 3d identiques
        //-- pour les ramener au meme parametre sur la
        //-- GLine
        if (VertexCount != 0)
        {
            double pf = (_fipt ? _svtx.Value(_indf).ParameterOnLine() : 0.0);
            double pl = (_lapt ? _svtx.Value(_indl).ParameterOnLine() : 0.0);
            double par = pnt.ParameterOnLine();
            if (ArcType() == PatchLineType.Circle || ArcType() == PatchLineType.Ellipse)
            {
                if (_fipt && _lapt)
                {
                    while (par < pf)
                    {
                        par += M_PI + M_PI;
                    }
                    while (par > pl)
                    {
                        par -= M_PI + M_PI;
                    }
                    if (par < pf)
                    {
                        double precisionPConfusion = global::Kerf.Engine.Foundation.Precision.PConfusion() * 1000.0;
                        if ((pf - par) > precisionPConfusion)
                        {
                            return;
                        }
                    }
                    PatchPoint parModif = pnt;
                    parModif.SetParameter(par);
                    _svtx.Append(parModif);
                    return;
                }
            }
            else
            {
                if (_fipt && _lapt)
                {
                    if (pl < par || par < pf)
                    {
                        return;
                    }
                }
            }
        }
        _svtx.Append(pnt);
    }

    /// <summary>
    /// OCCT's "replace": it appends <paramref name="pnt"/> and ignores <paramref name="index"/> (the replacement is
    /// commented out in OCCT).
    /// </summary>
    public void Replace(int index, in PatchPoint pnt)
    {
        _svtx.Append(pnt); // OCCT: Index is ignored, the point appended (see occt-issues.md)
        //--   svtx(Index) = Pnt;
    }

    public void SetFirstPoint(int indFirst)
    {
        _fipt = true;
        _indf = indFirst;
    }

    public void SetLastPoint(int indLast)
    {
        _lapt = true;
        _indl = indLast;
    }

    /// <summary>The line, when <see cref="PatchLine.ArcType"/> is IntPatch_Lin; throws Standard_DomainError otherwise.</summary>
    public Line3D Line()
    {
        if (_typ != PatchLineType.Lin)
        {
            throw new DomainErrorException();
        }
        return new Line3D(_pos.Axis);
    }

    /// <summary>The circle, when the type is IntPatch_Circle; throws Standard_DomainError otherwise.</summary>
    public Circle3D Circle()
    {
        if (_typ != PatchLineType.Circle)
        {
            throw new DomainErrorException();
        }
        return new Circle3D(_pos, _par1);
    }

    /// <summary>The ellipse, when the type is IntPatch_Ellipse; throws Standard_DomainError otherwise.</summary>
    public Ellipse3D Ellipse()
    {
        if (_typ != PatchLineType.Ellipse)
        {
            throw new DomainErrorException();
        }
        return new Ellipse3D(_pos, _par1, _par2);
    }

    /// <summary>The parabola, when the type is IntPatch_Parabola; throws Standard_DomainError otherwise.</summary>
    public Parabola3D Parabola()
    {
        if (_typ != PatchLineType.Parabola)
        {
            throw new DomainErrorException();
        }
        return new Parabola3D(_pos, _par1);
    }

    /// <summary>The hyperbola, when the type is IntPatch_Hyperbola; throws Standard_DomainError otherwise.</summary>
    public Hyperbola3D Hyperbola()
    {
        if (_typ != PatchLineType.Hyperbola)
        {
            throw new DomainErrorException();
        }
        return new Hyperbola3D(_pos, _par1, _par2);
    }

    /// <summary>True if the line has a known first point (<see cref="FirstPoint"/>).</summary>
    public bool HasFirstPoint => _fipt;

    /// <summary>True if the line has a known last point (<see cref="LastPoint"/>).</summary>
    public bool HasLastPoint => _lapt;

    /// <summary>The first point; throws Standard_DomainError if there is none.</summary>
    public PatchPoint FirstPoint()
    {
        if (!_fipt)
        {
            throw new DomainErrorException();
        }
        return _svtx.Value(_indf);
    }

    /// <summary>The last point; throws Standard_DomainError if there is none.</summary>
    public PatchPoint LastPoint()
    {
        if (!_lapt)
        {
            throw new DomainErrorException();
        }
        return _svtx.Value(_indl);
    }

    public int VertexCount => _svtx.Length();

    /// <summary>The vertex of range <paramref name="index"/>.</summary>
    public PatchPoint Vertex(int index) => _svtx.Value(index);

    /// <summary>
    /// Sets the parameters of all the vertices on the line: removes the duplicates on the restrictions, sorts the
    /// vertices by parameter, distinguishes the seam points of a closed conic, then makes the first and last
    /// vertices the first and last points.
    /// </summary>
    public void ComputeVertexParameters(double tol)
    {
        bool sortIsOK, aPointDeleted;
        bool sortAgain = true;
        int i, j;
        double paramMinOnLine = (_fipt ? Vertex(_indf).ParameterOnLine() : -100000.0);
        double paramMaxOnLine = (_lapt ? Vertex(_indl).ParameterOnLine() : 100000.0);

        //----------------------------------------------------------
        //--     F i l t r e   s u r   r e s t r i c t i o n s   --
        //----------------------------------------------------------
        //-- deux vertex sur la meme restriction et seulement
        //-- sur celle ci ne doivent pas avoir le meme parametre
        //==========================================================
        //-- 2 vertices on the same restriction and only
        //-- on that one must not have the same parametres

        int nbvtx = VertexCount;

        double precisionPConfusion = global::Kerf.Engine.Foundation.Precision.PConfusion() * 1000.0;

        do
        {
            aPointDeleted = false;
            for (i = 1; (i <= nbvtx) && (!aPointDeleted); i++)
            {
                PatchPoint vtXi = _svtx.Value(i);
                if (vtXi.IsOnDomS1 || vtXi.IsOnDomS2)
                {
                    for (j = 1; (j <= nbvtx) && (!aPointDeleted); j++)
                    {
                        if (i != j)
                        {
                            PatchPoint vtXj = _svtx.Value(j);
                            if ((!vtXj.IsOnDomS1) && (!vtXj.IsOnDomS2))
                            {
                                if (Math.Abs(vtXi.ParameterOnLine() - vtXj.ParameterOnLine()) <= precisionPConfusion)
                                {
                                    _svtx.Remove(j);
                                    nbvtx--;
                                    if (_lapt)
                                    {
                                        if (_indl > j)
                                        {
                                            _indl--;
                                        }
                                    }
                                    if (_fipt)
                                    {
                                        if (_indf > j)
                                        {
                                            _indf--;
                                        }
                                    }
                                    aPointDeleted = true;
                                }
                            }
                        }
                    }
                }
            }
        } while (aPointDeleted && nbvtx > 2);

        do
        {
            aPointDeleted = false;
            for (i = 1; (i <= nbvtx) && (!aPointDeleted); i++)
            {
                PatchPoint vtXi = _svtx.Value(i);
                if (vtXi.IsOnDomS1 && (!vtXi.IsOnDomS2))
                {
                    for (j = 1; (j <= nbvtx) && (!aPointDeleted); j++)
                    {
                        if (i != j)
                        {
                            PatchPoint vtXj = _svtx.Value(j);
                            if (vtXj.IsOnDomS1 && (!vtXj.IsOnDomS2))
                            {
                                if (Math.Abs(vtXi.ParameterOnArc1() - vtXj.ParameterOnArc1()) <= precisionPConfusion)
                                {
                                    if (ReferenceEquals(vtXi.ArcOnS1(), vtXj.ArcOnS1()))
                                    {
                                        if (vtXi.IsVertexOnS1)
                                        {
                                            RemoveDuplicate(j, ref nbvtx);
                                        }
                                        else
                                        {
                                            RemoveDuplicate(i, ref nbvtx);
                                        }
                                        aPointDeleted = true;
                                    }
                                }
                            }
                        }
                    }
                }
            }
        } while (aPointDeleted);

        do
        {
            aPointDeleted = false;
            for (i = 1; (i <= nbvtx) && (!aPointDeleted); i++)
            {
                PatchPoint vtXi = _svtx.Value(i);
                if (vtXi.IsOnDomS2 && (!vtXi.IsOnDomS1))
                {
                    for (j = 1; (j <= nbvtx) && (!aPointDeleted); j++)
                    {
                        if (i != j)
                        {
                            PatchPoint vtXj = _svtx.Value(j);
                            if (vtXj.IsOnDomS2 && (!vtXj.IsOnDomS1))
                            {
                                if (Math.Abs(vtXi.ParameterOnArc2() - vtXj.ParameterOnArc2()) <= precisionPConfusion)
                                {
                                    if (ReferenceEquals(vtXi.ArcOnS2(), vtXj.ArcOnS2()))
                                    {
                                        // OCCT: tests IsVertexOnS1 on points that are on S2 only (always false), so
                                        // the point i is the one removed (see occt-issues.md)
                                        if (vtXi.IsVertexOnS1)
                                        {
                                            RemoveDuplicate(j, ref nbvtx);
                                        }
                                        else
                                        {
                                            RemoveDuplicate(i, ref nbvtx);
                                        }
                                        aPointDeleted = true;
                                    }
                                }
                            }
                        }
                    }
                }
            }
        } while (aPointDeleted);

        //----------------------------------------------------------
        //-- Tri des vertex et suppression des Vtx superflus
        //--
        //// modified by jgv, 2.11.01 for BUC61033 ////
        double u1min = RealLast(), u1max = RealFirst();
        double u2min = RealLast(), u2max = RealFirst();
        bool toBreak = false;
        ///////////////////////////////////////////////
        do
        {
            nbvtx = VertexCount;
            if (sortAgain)
            {
                do
                {
                    sortIsOK = true;
                    for (i = 2; i <= nbvtx; i++)
                    {
                        if (_svtx.Value(i - 1).ParameterOnLine() > _svtx.Value(i).ParameterOnLine())
                        {
                            sortIsOK = false;
                            _svtx.Exchange(i - 1, i);
                            if (_fipt)
                            {
                                if (_indf == i)
                                {
                                    _indf = i - 1;
                                }
                                else if (_indf == i - 1)
                                {
                                    _indf = i;
                                }
                            }
                            if (_lapt)
                            {
                                if (_indl == i)
                                {
                                    _indl = i - 1;
                                }
                                else if (_indl == i - 1)
                                {
                                    _indl = i;
                                }
                            }
                        }
                    }
                } while (!sortIsOK);
            }

            //// modified by jgv, 2.11.01 for BUC61033 ////
            if (toBreak)
            {
                break;
            }
            ///////////////////////////////////////////////

            sortAgain = false;
            sortIsOK = true;
            for (i = 2; i <= nbvtx && sortIsOK; i++)
            {
                ref PatchPoint vtx = ref _svtx.ChangeValue(i);
                for (j = 1; j <= nbvtx && sortIsOK; j++)
                {
                    if (i != j)
                    {
                        ref PatchPoint vtxm1 = ref _svtx.ChangeValue(j);
                        bool kill = false;
                        bool killm1 = false;
                        if (Math.Abs(vtxm1.ParameterOnLine() - vtx.ParameterOnLine()) < precisionPConfusion)
                        {
                            if (vtxm1.IsOnDomS1 && vtx.IsOnDomS1) //-- OnS1    OnS1
                            {
                                if (ReferenceEquals(vtxm1.ArcOnS1(), vtx.ArcOnS1())) //-- OnS1 == OnS1
                                {
                                    if (vtxm1.IsOnDomS2) //-- OnS1 == OnS1  OnS2
                                    {
                                        if (!vtx.IsOnDomS2) //-- OnS1 == OnS1  OnS2 PasOnS2
                                        {
                                            kill = true;
                                        }
                                        else
                                        {
                                            if (ReferenceEquals(vtxm1.ArcOnS2(), vtx.ArcOnS2())) //-- OnS1 == OnS1  OnS2 == OnS2
                                            {
                                                if (vtxm1.IsVertexOnS2)
                                                {
                                                    kill = true;
                                                }
                                                else
                                                {
                                                    killm1 = true;
                                                }
                                            }
                                        }
                                    }
                                    else //-- OnS1 == OnS1  PasOnS2
                                    {
                                        if (vtx.IsOnDomS2) //-- OnS1 == OnS1  PasOnS2  OnS2
                                        {
                                            killm1 = true;
                                        }
                                    }
                                }
                            }
                            else //-- Pas OnS1  et  OnS1
                            {
                                if (!vtxm1.IsOnDomS2 && !vtx.IsOnDomS2)
                                {
                                    if (vtxm1.IsOnDomS1 && !vtx.IsOnDomS1)
                                    {
                                        kill = true;
                                    }
                                    else if (vtx.IsOnDomS1 && !vtxm1.IsOnDomS1)
                                    {
                                        killm1 = true;
                                    }
                                }
                            }

                            if (!(kill || killm1))
                            {
                                if (vtxm1.IsOnDomS2 && vtx.IsOnDomS2) //-- OnS2    OnS2
                                {
                                    if (ReferenceEquals(vtxm1.ArcOnS2(), vtx.ArcOnS2())) //-- OnS2 == OnS2
                                    {
                                        if (vtxm1.IsOnDomS1) //-- OnS2 == OnS2  OnS1
                                        {
                                            if (!vtx.IsOnDomS1) //-- OnS2 == OnS2  OnS1 PasOnS1
                                            {
                                                kill = true;
                                            }
                                            else
                                            {
                                                if (ReferenceEquals(vtxm1.ArcOnS1(), vtx.ArcOnS1())) //-- OnS2 == OnS2  OnS1 == OnS1
                                                {
                                                    if (vtxm1.IsVertexOnS1)
                                                    {
                                                        kill = true; //-- OnS2 == OnS2  OnS1 == OnS1  Vtx PasVtx
                                                    }
                                                    else
                                                    {
                                                        killm1 = true; //-- OnS2 == OnS2  OnS1 == OnS1  PasVtx Vtx
                                                    }
                                                }
                                            }
                                        }
                                        else
                                        { //-- OnS2 == OnS2  PasOnS1
                                            if (vtx.IsOnDomS1) //-- OnS2 == OnS2  PasOnS1  OnS1
                                            {
                                                killm1 = true;
                                            }
                                        }
                                    }
                                }
                                else //-- Pas OnS2  et  OnS2
                                {
                                    if (!vtxm1.IsOnDomS1 && !vtx.IsOnDomS1)
                                    {
                                        if (vtxm1.IsOnDomS2 && !vtx.IsOnDomS2)
                                        {
                                            kill = true;
                                        }
                                        else if (vtx.IsOnDomS2 && !vtxm1.IsOnDomS2)
                                        {
                                            killm1 = true;
                                        }
                                    }
                                }
                            }

                            //-- On a j < i
                            if (kill)
                            {
                                sortIsOK = false;
                                if (_lapt)
                                {
                                    if (_indl > i)
                                    {
                                        _indl--;
                                    }
                                    else if (_indl == i)
                                    {
                                        _indl = j;
                                    }
                                }

                                if (_fipt)
                                {
                                    if (_indf > i)
                                    {
                                        _indf--;
                                    }
                                    else if (_indf == i)
                                    {
                                        _indf = j;
                                    }
                                }

                                _svtx.Remove(i);
                                nbvtx--;
                            }
                            else if (killm1)
                            {
                                sortIsOK = false;
                                if (_lapt)
                                {
                                    if (_indl > j)
                                    {
                                        _indl--;
                                    }
                                    else if (_indl == j)
                                    {
                                        _indl = i - 1;
                                    }
                                }

                                if (_fipt)
                                {
                                    if (_indf > j)
                                    {
                                        _indf--;
                                    }
                                    else if (_indf == j)
                                    {
                                        _indf = i - 1;
                                    }
                                }

                                _svtx.Remove(j);
                                nbvtx--;
                            }
                            else if (ArcType() == PatchLineType.Circle || ArcType() == PatchLineType.Ellipse) // eap
                            {
                                //-- deux points de meme parametre qui ne peuvent etre confondus
                                //-- On change les parametres d un des points si les points UV sont
                                //-- differents. Ceci distingue le cas des aretes de couture.
                                // ==========================================================
                                //-- 2 points with the same parameters
                                //-- Change parametres of one point if points UV are
                                //-- different. This is the case of seam edge

                                double ponline = vtx.ParameterOnLine();
                                // eap, =>>
                                double newParam = ponline;
                                double piPi = M_PI + M_PI;
                                bool is2PI = (Math.Abs(ponline - piPi) <= precisionPConfusion);

                                if (nbvtx > 2 && // do this check if seam edge only gives vertices
                                    !is2PI)
                                { // but always change 2PI -> 0
                                    continue;
                                }

                                if (is2PI)
                                {
                                    newParam = 0;
                                }
                                else if (Math.Abs(ponline) <= precisionPConfusion)
                                {
                                    newParam = piPi;
                                }
                                else
                                {
                                    newParam -= piPi;
                                }

                                vtxm1.Parameters(out double u1a, out double v1a, out double u2a, out double v2a);
                                vtx.Parameters(out double u1b, out double v1b, out double u2b, out double v2b);
                                int flag = 0;

                                if ((Math.Abs(u1a - u1b) <= precisionPConfusion))
                                {
                                    flag |= 1;
                                }

                                if ((Math.Abs(v1a - v1b) <= precisionPConfusion))
                                {
                                    flag |= 2;
                                }
                                if ((Math.Abs(u2a - u2b) <= precisionPConfusion))
                                {
                                    flag |= 4;
                                }

                                if ((Math.Abs(v2a - v2b) <= precisionPConfusion))
                                {
                                    flag |= 8;
                                }

                                bool testOn1 = false;
                                bool testOn2 = false;

                                switch (flag)
                                {
                                    case 3: //-- meme point U1 V1
                                    case 7: //-- meme point U1 V1   meme U2
                                    case 12: //--                    meme U2 V2
                                    case 13: //-- meme point U1      meme U2 V2
                                    case 10: //-- meme point    V1   meme    V2   Test si U1a=U1b Mod 2PI et Test si
                                             // U2a=U2b Mod 2PI
                                        break;
                                    case 11: //-- meme point U1 V1   meme    V2   Test si U2a=U2b Mod 2PI
                                    {
                                        testOn2 = true;
                                        break;
                                    }

                                    case 14: //-- meme point    V1   meme U2 V2   Test si U1a=U1b Mod 2PI
                                    {
                                        testOn1 = true;
                                        break;
                                    }
                                    default:
                                        break;
                                }

                                // eap
                                // if(ArcType()==IntPatch_Circle || ArcType()==IntPatch_Ellipse) {}
                                if (testOn1)
                                {
                                    //// modified by jgv, 2.11.01 for BUC61033 ////
                                    double u1A = (u1a < u1b) ? u1a : u1b;
                                    double u1B = (u1a < u1b) ? u1b : u1a;

                                    if (u1min == RealLast())
                                    {
                                        u1min = u1A;
                                        u1max = u1B;
                                    }
                                    else
                                    {
                                        if (Math.Abs(u1A - u1min) > precisionPConfusion)
                                        {
                                            toBreak = true;
                                        }
                                        if (Math.Abs(u1B - u1max) > precisionPConfusion)
                                        {
                                            toBreak = true;
                                        }
                                    }
                                    ///////////////////////////////////////////////
                                    if (newParam >= paramMinOnLine && newParam <= paramMaxOnLine)
                                    {
                                        sortAgain = true;
                                        sortIsOK = false;
                                        if (newParam > ponline)
                                        {
                                            if (u1a < u1b)
                                            {
                                                vtx.SetParameter(newParam);
                                            }
                                            else
                                            {
                                                vtxm1.SetParameter(newParam);
                                            }
                                        }
                                        else
                                        {
                                            if (u1a > u1b)
                                            {
                                                vtx.SetParameter(newParam);
                                            }
                                            else
                                            {
                                                vtxm1.SetParameter(newParam);
                                            }
                                        }
                                    }
                                }

                                if (testOn2)
                                {
                                    //// modified by jgv, 2.11.01 for BUC61033 ////
                                    double u2A = (u2a < u2b) ? u2a : u2b;
                                    double u2B = (u2a < u2b) ? u2b : u2a;
                                    if (u2min == RealLast())
                                    {
                                        u2min = u2A;
                                        u2max = u2B;
                                    }
                                    else
                                    {
                                        if (Math.Abs(u2A - u2min) > precisionPConfusion)
                                        {
                                            toBreak = true;
                                        }

                                        if (Math.Abs(u2B - u2max) > precisionPConfusion)
                                        {
                                            toBreak = true;
                                        }
                                    }
                                    ///////////////////////////////////////////////
                                    if (newParam >= paramMinOnLine && newParam <= paramMaxOnLine)
                                    {
                                        sortAgain = true;
                                        sortIsOK = false;
                                        if (newParam > ponline)
                                        {
                                            if (u2a < u2b)
                                            {
                                                vtx.SetParameter(newParam);
                                            }
                                            else
                                            {
                                                vtxm1.SetParameter(newParam);
                                            }
                                        }
                                        else
                                        {
                                            if (u2a > u2b)
                                            {
                                                vtx.SetParameter(newParam);
                                            }
                                            else
                                            {
                                                vtxm1.SetParameter(newParam);
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                } //-- if(i!=j)
            }
        } while (!sortIsOK);

        //-- Recalcul de fipt et lapt
        //--
        nbvtx = VertexCount;
        if (nbvtx != 0)
        {
            do
            {
                sortIsOK = true;
                for (i = 2; i <= nbvtx; i++)
                {
                    if (_svtx.Value(i - 1).ParameterOnLine() > _svtx.Value(i).ParameterOnLine())
                    {
                        sortIsOK = false;
                        _svtx.Exchange(i - 1, i);
                    }
                }
            } while (!sortIsOK);

            _indl = nbvtx;
            _indf = 1;
        }
    }

    // The removal of a duplicate on a restriction, with the first and last indices updated, that OCCT writes out four
    // times in the filters of ComputeVertexParameters.
    private void RemoveDuplicate(int index, ref int nbVtx)
    {
        _svtx.Remove(index);
        nbVtx--;
        if (_lapt)
        {
            if (_indl > index)
            {
                _indl--;
            }
        }
        if (_fipt)
        {
            if (_indf > index)
            {
                _indf--;
            }
        }
    }
}
