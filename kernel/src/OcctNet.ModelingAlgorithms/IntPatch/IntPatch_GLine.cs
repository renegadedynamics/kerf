// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_GLine.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_GLine.lxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_GLine.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Geometry;
using static Occt.IntPatch_IType;

namespace Occt;

/// <summary>
/// An intersection line that is a conic (a line, circle, ellipse, parabola or hyperbola), with its vertices.
/// </summary>
/// <remarks>
/// A working object (decision 609). The conic is kept as OCCT keeps it: its position (a gp_Ax2) and up to two
/// parameters (radius, radii or focal length); a line's position is the plane through it normal to it.
/// <see cref="Vertex"/>, <see cref="FirstPoint"/> and <see cref="LastPoint"/> return copies (OCCT's const
/// references). OCCT's <c>Replace</c> appends the point (its index is ignored).
/// </remarks>
public class IntPatch_GLine : IntPatch_Line
{
    private Frame3D pos;
    private double par1;
    private double par2;
    private bool fipt;
    private bool lapt;
    private int indf;
    private int indl;
    private readonly NCollection_Sequence<IntPatch_Point> svtx = new();

    /// <summary>Creates a line as intersection line when the transitions are in or out.</summary>
    public IntPatch_GLine(in Line3D L, bool Tang, IntSurf_TypeTrans Trans1, IntSurf_TypeTrans Trans2)
        : base(Tang, Trans1, Trans2)
    {
        par1 = 0.0;
        par2 = 0.0;
        typ = IntPatch_Lin;
        pos = new Plane3D(L.Location(), L.Direction()).Position().Ax2();
    }

    /// <summary>Creates a line as intersection line when the transitions are touch.</summary>
    public IntPatch_GLine(in Line3D L, bool Tang, IntSurf_Situation Situ1, IntSurf_Situation Situ2)
        : base(Tang, Situ1, Situ2)
    {
        par1 = 0.0;
        par2 = 0.0;
        typ = IntPatch_Lin;
        pos = new Plane3D(L.Location(), L.Direction()).Position().Ax2();
    }

    /// <summary>Creates a line as intersection line when the transitions are undecided.</summary>
    public IntPatch_GLine(in Line3D L, bool Tang)
        : base(Tang)
    {
        par1 = 0.0;
        par2 = 0.0;
        typ = IntPatch_Lin;
        pos = new Plane3D(L.Location(), L.Direction()).Position().Ax2();
    }

    /// <summary>Creates a circle as intersection line when the transitions are in or out.</summary>
    public IntPatch_GLine(in Circle3D C, bool Tang, IntSurf_TypeTrans Trans1, IntSurf_TypeTrans Trans2)
        : base(Tang, Trans1, Trans2)
    {
        pos = C.Position();
        par1 = C.Radius();
        par2 = 0.0;
        typ = IntPatch_Circle;
    }

    /// <summary>Creates a circle as intersection line when the transitions are touch.</summary>
    public IntPatch_GLine(in Circle3D C, bool Tang, IntSurf_Situation Situ1, IntSurf_Situation Situ2)
        : base(Tang, Situ1, Situ2)
    {
        pos = C.Position();
        par1 = C.Radius();
        par2 = 0.0;
        typ = IntPatch_Circle;
    }

    /// <summary>Creates a circle as intersection line when the transitions are undecided.</summary>
    public IntPatch_GLine(in Circle3D C, bool Tang)
        : base(Tang)
    {
        pos = C.Position();
        par1 = C.Radius();
        par2 = 0.0;
        typ = IntPatch_Circle;
    }

    /// <summary>Creates an ellipse as intersection line when the transitions are in or out.</summary>
    public IntPatch_GLine(in Ellipse3D E, bool Tang, IntSurf_TypeTrans Trans1, IntSurf_TypeTrans Trans2)
        : base(Tang, Trans1, Trans2)
    {
        pos = E.Position();
        par1 = E.MajorRadius();
        par2 = E.MinorRadius();
        typ = IntPatch_Ellipse;
    }

    /// <summary>Creates an ellipse as intersection line when the transitions are touch.</summary>
    public IntPatch_GLine(in Ellipse3D E, bool Tang, IntSurf_Situation Situ1, IntSurf_Situation Situ2)
        : base(Tang, Situ1, Situ2)
    {
        pos = E.Position();
        par1 = E.MajorRadius();
        par2 = E.MinorRadius();
        typ = IntPatch_Ellipse;
    }

    /// <summary>Creates an ellipse as intersection line when the transitions are undecided.</summary>
    public IntPatch_GLine(in Ellipse3D E, bool Tang)
        : base(Tang)
    {
        pos = E.Position();
        par1 = E.MajorRadius();
        par2 = E.MinorRadius();
        typ = IntPatch_Ellipse;
    }

    /// <summary>Creates a parabola as intersection line when the transitions are in or out.</summary>
    public IntPatch_GLine(in Parabola3D P, bool Tang, IntSurf_TypeTrans Trans1, IntSurf_TypeTrans Trans2)
        : base(Tang, Trans1, Trans2)
    {
        pos = P.Position();
        par1 = P.Focal();
        par2 = 0.0;
        typ = IntPatch_Parabola;
    }

    /// <summary>Creates a parabola as intersection line when the transitions are touch.</summary>
    public IntPatch_GLine(in Parabola3D P, bool Tang, IntSurf_Situation Situ1, IntSurf_Situation Situ2)
        : base(Tang, Situ1, Situ2)
    {
        pos = P.Position();
        par1 = P.Focal();
        par2 = 0.0;
        typ = IntPatch_Parabola;
    }

    /// <summary>Creates a parabola as intersection line when the transitions are undecided.</summary>
    public IntPatch_GLine(in Parabola3D P, bool Tang)
        : base(Tang)
    {
        pos = P.Position();
        par1 = P.Focal();
        par2 = 0.0;
        typ = IntPatch_Parabola;
    }

    /// <summary>Creates a hyperbola as intersection line when the transitions are in or out.</summary>
    public IntPatch_GLine(in Hyperbola3D H, bool Tang, IntSurf_TypeTrans Trans1, IntSurf_TypeTrans Trans2)
        : base(Tang, Trans1, Trans2)
    {
        pos = H.Position();
        par1 = H.MajorRadius();
        par2 = H.MinorRadius();
        typ = IntPatch_Hyperbola;
    }

    /// <summary>Creates a hyperbola as intersection line when the transitions are touch.</summary>
    public IntPatch_GLine(in Hyperbola3D H, bool Tang, IntSurf_Situation Situ1, IntSurf_Situation Situ2)
        : base(Tang, Situ1, Situ2)
    {
        pos = H.Position();
        par1 = H.MajorRadius();
        par2 = H.MinorRadius();
        typ = IntPatch_Hyperbola;
    }

    /// <summary>Creates a hyperbola as intersection line when the transitions are undecided.</summary>
    public IntPatch_GLine(in Hyperbola3D H, bool Tang)
        : base(Tang)
    {
        pos = H.Position();
        par1 = H.MajorRadius();
        par2 = H.MinorRadius();
        typ = IntPatch_Hyperbola;
    }

    /// <summary>Adds a vertex; on a closed conic whose first and last points are known, brought into their range.</summary>
    public void AddVertex(in IntPatch_Point Pnt)
    {
        //-- On detecte le cas de 2 points 3d identiques
        //-- pour les ramener au meme parametre sur la
        //-- GLine
        if (NbVertex() != 0)
        {
            double pf = (fipt ? svtx.Value(indf).ParameterOnLine() : 0.0);
            double pl = (lapt ? svtx.Value(indl).ParameterOnLine() : 0.0);
            double par = Pnt.ParameterOnLine();
            if (ArcType() == IntPatch_Circle || ArcType() == IntPatch_Ellipse)
            {
                if (fipt && lapt)
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
                        double PrecisionPConfusion = global::Kerf.Engine.Foundation.Precision.PConfusion() * 1000.0;
                        if ((pf - par) > PrecisionPConfusion)
                        {
                            return;
                        }
                    }
                    IntPatch_Point ParModif = Pnt;
                    ParModif.SetParameter(par);
                    svtx.Append(ParModif);
                    return;
                }
            }
            else
            {
                if (fipt && lapt)
                {
                    if (pl < par || par < pf)
                    {
                        return;
                    }
                }
            }
        }
        svtx.Append(Pnt);
    }

    /// <summary>
    /// OCCT's "replace": it appends <paramref name="Pnt"/> and ignores <paramref name="Index"/> (the replacement is
    /// commented out in OCCT).
    /// </summary>
    public void Replace(int Index, in IntPatch_Point Pnt)
    {
        svtx.Append(Pnt); // OCCT: Index is ignored, the point appended (see occt-issues.md)
        //--   svtx(Index) = Pnt;
    }

    public void SetFirstPoint(int IndFirst)
    {
        fipt = true;
        indf = IndFirst;
    }

    public void SetLastPoint(int IndLast)
    {
        lapt = true;
        indl = IndLast;
    }

    /// <summary>The line, when <see cref="IntPatch_Line.ArcType"/> is IntPatch_Lin; throws Standard_DomainError otherwise.</summary>
    public Line3D Line()
    {
        if (typ != IntPatch_Lin)
        {
            throw new DomainErrorException();
        }
        return new Line3D(pos.Axis());
    }

    /// <summary>The circle, when the type is IntPatch_Circle; throws Standard_DomainError otherwise.</summary>
    public Circle3D Circle()
    {
        if (typ != IntPatch_Circle)
        {
            throw new DomainErrorException();
        }
        return new Circle3D(pos, par1);
    }

    /// <summary>The ellipse, when the type is IntPatch_Ellipse; throws Standard_DomainError otherwise.</summary>
    public Ellipse3D Ellipse()
    {
        if (typ != IntPatch_Ellipse)
        {
            throw new DomainErrorException();
        }
        return new Ellipse3D(pos, par1, par2);
    }

    /// <summary>The parabola, when the type is IntPatch_Parabola; throws Standard_DomainError otherwise.</summary>
    public Parabola3D Parabola()
    {
        if (typ != IntPatch_Parabola)
        {
            throw new DomainErrorException();
        }
        return new Parabola3D(pos, par1);
    }

    /// <summary>The hyperbola, when the type is IntPatch_Hyperbola; throws Standard_DomainError otherwise.</summary>
    public Hyperbola3D Hyperbola()
    {
        if (typ != IntPatch_Hyperbola)
        {
            throw new DomainErrorException();
        }
        return new Hyperbola3D(pos, par1, par2);
    }

    /// <summary>True if the line has a known first point (<see cref="FirstPoint"/>).</summary>
    public bool HasFirstPoint() => fipt;

    /// <summary>True if the line has a known last point (<see cref="LastPoint"/>).</summary>
    public bool HasLastPoint() => lapt;

    /// <summary>The first point; throws Standard_DomainError if there is none.</summary>
    public IntPatch_Point FirstPoint()
    {
        if (!fipt)
        {
            throw new DomainErrorException();
        }
        return svtx.Value(indf);
    }

    /// <summary>The last point; throws Standard_DomainError if there is none.</summary>
    public IntPatch_Point LastPoint()
    {
        if (!lapt)
        {
            throw new DomainErrorException();
        }
        return svtx.Value(indl);
    }

    public int NbVertex() => svtx.Length();

    /// <summary>The vertex of range <paramref name="Index"/>.</summary>
    public IntPatch_Point Vertex(int Index) => svtx.Value(Index);

    /// <summary>
    /// Sets the parameters of all the vertices on the line: removes the duplicates on the restrictions, sorts the
    /// vertices by parameter, distinguishes the seam points of a closed conic, then makes the first and last
    /// vertices the first and last points.
    /// </summary>
    public void ComputeVertexParameters(double Tol)
    {
        bool SortIsOK, APointDeleted;
        bool SortAgain = true;
        int i, j;
        double ParamMinOnLine = (fipt ? Vertex(indf).ParameterOnLine() : -100000.0);
        double ParamMaxOnLine = (lapt ? Vertex(indl).ParameterOnLine() : 100000.0);

        //----------------------------------------------------------
        //--     F i l t r e   s u r   r e s t r i c t i o n s   --
        //----------------------------------------------------------
        //-- deux vertex sur la meme restriction et seulement
        //-- sur celle ci ne doivent pas avoir le meme parametre
        //==========================================================
        //-- 2 vertices on the same restriction and only
        //-- on that one must not have the same parametres

        int nbvtx = NbVertex();

        double PrecisionPConfusion = global::Kerf.Engine.Foundation.Precision.PConfusion() * 1000.0;

        do
        {
            APointDeleted = false;
            for (i = 1; (i <= nbvtx) && (!APointDeleted); i++)
            {
                IntPatch_Point VTXi = svtx.Value(i);
                if (VTXi.IsOnDomS1() || VTXi.IsOnDomS2())
                {
                    for (j = 1; (j <= nbvtx) && (!APointDeleted); j++)
                    {
                        if (i != j)
                        {
                            IntPatch_Point VTXj = svtx.Value(j);
                            if ((!VTXj.IsOnDomS1()) && (!VTXj.IsOnDomS2()))
                            {
                                if (Math.Abs(VTXi.ParameterOnLine() - VTXj.ParameterOnLine()) <= PrecisionPConfusion)
                                {
                                    svtx.Remove(j);
                                    nbvtx--;
                                    if (lapt)
                                    {
                                        if (indl > j)
                                        {
                                            indl--;
                                        }
                                    }
                                    if (fipt)
                                    {
                                        if (indf > j)
                                        {
                                            indf--;
                                        }
                                    }
                                    APointDeleted = true;
                                }
                            }
                        }
                    }
                }
            }
        } while (APointDeleted && nbvtx > 2);

        do
        {
            APointDeleted = false;
            for (i = 1; (i <= nbvtx) && (!APointDeleted); i++)
            {
                IntPatch_Point VTXi = svtx.Value(i);
                if (VTXi.IsOnDomS1() && (!VTXi.IsOnDomS2()))
                {
                    for (j = 1; (j <= nbvtx) && (!APointDeleted); j++)
                    {
                        if (i != j)
                        {
                            IntPatch_Point VTXj = svtx.Value(j);
                            if (VTXj.IsOnDomS1() && (!VTXj.IsOnDomS2()))
                            {
                                if (Math.Abs(VTXi.ParameterOnArc1() - VTXj.ParameterOnArc1()) <= PrecisionPConfusion)
                                {
                                    if (ReferenceEquals(VTXi.ArcOnS1(), VTXj.ArcOnS1()))
                                    {
                                        if (VTXi.IsVertexOnS1())
                                        {
                                            RemoveDuplicate(j, ref nbvtx);
                                        }
                                        else
                                        {
                                            RemoveDuplicate(i, ref nbvtx);
                                        }
                                        APointDeleted = true;
                                    }
                                }
                            }
                        }
                    }
                }
            }
        } while (APointDeleted);

        do
        {
            APointDeleted = false;
            for (i = 1; (i <= nbvtx) && (!APointDeleted); i++)
            {
                IntPatch_Point VTXi = svtx.Value(i);
                if (VTXi.IsOnDomS2() && (!VTXi.IsOnDomS1()))
                {
                    for (j = 1; (j <= nbvtx) && (!APointDeleted); j++)
                    {
                        if (i != j)
                        {
                            IntPatch_Point VTXj = svtx.Value(j);
                            if (VTXj.IsOnDomS2() && (!VTXj.IsOnDomS1()))
                            {
                                if (Math.Abs(VTXi.ParameterOnArc2() - VTXj.ParameterOnArc2()) <= PrecisionPConfusion)
                                {
                                    if (ReferenceEquals(VTXi.ArcOnS2(), VTXj.ArcOnS2()))
                                    {
                                        // OCCT: tests IsVertexOnS1 on points that are on S2 only (always false), so
                                        // the point i is the one removed (see occt-issues.md)
                                        if (VTXi.IsVertexOnS1())
                                        {
                                            RemoveDuplicate(j, ref nbvtx);
                                        }
                                        else
                                        {
                                            RemoveDuplicate(i, ref nbvtx);
                                        }
                                        APointDeleted = true;
                                    }
                                }
                            }
                        }
                    }
                }
            }
        } while (APointDeleted);

        //----------------------------------------------------------
        //-- Tri des vertex et suppression des Vtx superflus
        //--
        //// modified by jgv, 2.11.01 for BUC61033 ////
        double u1min = RealLast(), u1max = RealFirst();
        double u2min = RealLast(), u2max = RealFirst();
        bool ToBreak = false;
        ///////////////////////////////////////////////
        do
        {
            nbvtx = NbVertex();
            if (SortAgain)
            {
                do
                {
                    SortIsOK = true;
                    for (i = 2; i <= nbvtx; i++)
                    {
                        if (svtx.Value(i - 1).ParameterOnLine() > svtx.Value(i).ParameterOnLine())
                        {
                            SortIsOK = false;
                            svtx.Exchange(i - 1, i);
                            if (fipt)
                            {
                                if (indf == i)
                                {
                                    indf = i - 1;
                                }
                                else if (indf == i - 1)
                                {
                                    indf = i;
                                }
                            }
                            if (lapt)
                            {
                                if (indl == i)
                                {
                                    indl = i - 1;
                                }
                                else if (indl == i - 1)
                                {
                                    indl = i;
                                }
                            }
                        }
                    }
                } while (!SortIsOK);
            }

            //// modified by jgv, 2.11.01 for BUC61033 ////
            if (ToBreak)
            {
                break;
            }
            ///////////////////////////////////////////////

            SortAgain = false;
            SortIsOK = true;
            for (i = 2; i <= nbvtx && SortIsOK; i++)
            {
                ref IntPatch_Point VTX = ref svtx.ChangeValue(i);
                for (j = 1; j <= nbvtx && SortIsOK; j++)
                {
                    if (i != j)
                    {
                        ref IntPatch_Point VTXM1 = ref svtx.ChangeValue(j);
                        bool kill = false;
                        bool killm1 = false;
                        if (Math.Abs(VTXM1.ParameterOnLine() - VTX.ParameterOnLine()) < PrecisionPConfusion)
                        {
                            if (VTXM1.IsOnDomS1() && VTX.IsOnDomS1()) //-- OnS1    OnS1
                            {
                                if (ReferenceEquals(VTXM1.ArcOnS1(), VTX.ArcOnS1())) //-- OnS1 == OnS1
                                {
                                    if (VTXM1.IsOnDomS2()) //-- OnS1 == OnS1  OnS2
                                    {
                                        if (!VTX.IsOnDomS2()) //-- OnS1 == OnS1  OnS2 PasOnS2
                                        {
                                            kill = true;
                                        }
                                        else
                                        {
                                            if (ReferenceEquals(VTXM1.ArcOnS2(), VTX.ArcOnS2())) //-- OnS1 == OnS1  OnS2 == OnS2
                                            {
                                                if (VTXM1.IsVertexOnS2())
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
                                        if (VTX.IsOnDomS2()) //-- OnS1 == OnS1  PasOnS2  OnS2
                                        {
                                            killm1 = true;
                                        }
                                    }
                                }
                            }
                            else //-- Pas OnS1  et  OnS1
                            {
                                if (!VTXM1.IsOnDomS2() && !VTX.IsOnDomS2())
                                {
                                    if (VTXM1.IsOnDomS1() && !VTX.IsOnDomS1())
                                    {
                                        kill = true;
                                    }
                                    else if (VTX.IsOnDomS1() && !VTXM1.IsOnDomS1())
                                    {
                                        killm1 = true;
                                    }
                                }
                            }

                            if (!(kill || killm1))
                            {
                                if (VTXM1.IsOnDomS2() && VTX.IsOnDomS2()) //-- OnS2    OnS2
                                {
                                    if (ReferenceEquals(VTXM1.ArcOnS2(), VTX.ArcOnS2())) //-- OnS2 == OnS2
                                    {
                                        if (VTXM1.IsOnDomS1()) //-- OnS2 == OnS2  OnS1
                                        {
                                            if (!VTX.IsOnDomS1()) //-- OnS2 == OnS2  OnS1 PasOnS1
                                            {
                                                kill = true;
                                            }
                                            else
                                            {
                                                if (ReferenceEquals(VTXM1.ArcOnS1(), VTX.ArcOnS1())) //-- OnS2 == OnS2  OnS1 == OnS1
                                                {
                                                    if (VTXM1.IsVertexOnS1())
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
                                            if (VTX.IsOnDomS1()) //-- OnS2 == OnS2  PasOnS1  OnS1
                                            {
                                                killm1 = true;
                                            }
                                        }
                                    }
                                }
                                else //-- Pas OnS2  et  OnS2
                                {
                                    if (!VTXM1.IsOnDomS1() && !VTX.IsOnDomS1())
                                    {
                                        if (VTXM1.IsOnDomS2() && !VTX.IsOnDomS2())
                                        {
                                            kill = true;
                                        }
                                        else if (VTX.IsOnDomS2() && !VTXM1.IsOnDomS2())
                                        {
                                            killm1 = true;
                                        }
                                    }
                                }
                            }

                            //-- On a j < i
                            if (kill)
                            {
                                SortIsOK = false;
                                if (lapt)
                                {
                                    if (indl > i)
                                    {
                                        indl--;
                                    }
                                    else if (indl == i)
                                    {
                                        indl = j;
                                    }
                                }

                                if (fipt)
                                {
                                    if (indf > i)
                                    {
                                        indf--;
                                    }
                                    else if (indf == i)
                                    {
                                        indf = j;
                                    }
                                }

                                svtx.Remove(i);
                                nbvtx--;
                            }
                            else if (killm1)
                            {
                                SortIsOK = false;
                                if (lapt)
                                {
                                    if (indl > j)
                                    {
                                        indl--;
                                    }
                                    else if (indl == j)
                                    {
                                        indl = i - 1;
                                    }
                                }

                                if (fipt)
                                {
                                    if (indf > j)
                                    {
                                        indf--;
                                    }
                                    else if (indf == j)
                                    {
                                        indf = i - 1;
                                    }
                                }

                                svtx.Remove(j);
                                nbvtx--;
                            }
                            else if (ArcType() == IntPatch_Circle || ArcType() == IntPatch_Ellipse) // eap
                            {
                                //-- deux points de meme parametre qui ne peuvent etre confondus
                                //-- On change les parametres d un des points si les points UV sont
                                //-- differents. Ceci distingue le cas des aretes de couture.
                                // ==========================================================
                                //-- 2 points with the same parameters
                                //-- Change parametres of one point if points UV are
                                //-- different. This is the case of seam edge

                                double ponline = VTX.ParameterOnLine();
                                // eap, =>>
                                double newParam = ponline;
                                double PiPi = M_PI + M_PI;
                                bool is2PI = (Math.Abs(ponline - PiPi) <= PrecisionPConfusion);

                                if (nbvtx > 2 && // do this check if seam edge only gives vertices
                                    !is2PI)
                                { // but always change 2PI -> 0
                                    continue;
                                }

                                if (is2PI)
                                {
                                    newParam = 0;
                                }
                                else if (Math.Abs(ponline) <= PrecisionPConfusion)
                                {
                                    newParam = PiPi;
                                }
                                else
                                {
                                    newParam -= PiPi;
                                }

                                VTXM1.Parameters(out double u1a, out double v1a, out double u2a, out double v2a);
                                VTX.Parameters(out double u1b, out double v1b, out double u2b, out double v2b);
                                int flag = 0;

                                if ((Math.Abs(u1a - u1b) <= PrecisionPConfusion))
                                {
                                    flag |= 1;
                                }

                                if ((Math.Abs(v1a - v1b) <= PrecisionPConfusion))
                                {
                                    flag |= 2;
                                }
                                if ((Math.Abs(u2a - u2b) <= PrecisionPConfusion))
                                {
                                    flag |= 4;
                                }

                                if ((Math.Abs(v2a - v2b) <= PrecisionPConfusion))
                                {
                                    flag |= 8;
                                }

                                bool TestOn1 = false;
                                bool TestOn2 = false;

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
                                        TestOn2 = true;
                                        break;
                                    }

                                    case 14: //-- meme point    V1   meme U2 V2   Test si U1a=U1b Mod 2PI
                                    {
                                        TestOn1 = true;
                                        break;
                                    }
                                    default:
                                        break;
                                }

                                // eap
                                // if(ArcType()==IntPatch_Circle || ArcType()==IntPatch_Ellipse) {}
                                if (TestOn1)
                                {
                                    //// modified by jgv, 2.11.01 for BUC61033 ////
                                    double U1A = (u1a < u1b) ? u1a : u1b;
                                    double U1B = (u1a < u1b) ? u1b : u1a;

                                    if (u1min == RealLast())
                                    {
                                        u1min = U1A;
                                        u1max = U1B;
                                    }
                                    else
                                    {
                                        if (Math.Abs(U1A - u1min) > PrecisionPConfusion)
                                        {
                                            ToBreak = true;
                                        }
                                        if (Math.Abs(U1B - u1max) > PrecisionPConfusion)
                                        {
                                            ToBreak = true;
                                        }
                                    }
                                    ///////////////////////////////////////////////
                                    if (newParam >= ParamMinOnLine && newParam <= ParamMaxOnLine)
                                    {
                                        SortAgain = true;
                                        SortIsOK = false;
                                        if (newParam > ponline)
                                        {
                                            if (u1a < u1b)
                                            {
                                                VTX.SetParameter(newParam);
                                            }
                                            else
                                            {
                                                VTXM1.SetParameter(newParam);
                                            }
                                        }
                                        else
                                        {
                                            if (u1a > u1b)
                                            {
                                                VTX.SetParameter(newParam);
                                            }
                                            else
                                            {
                                                VTXM1.SetParameter(newParam);
                                            }
                                        }
                                    }
                                }

                                if (TestOn2)
                                {
                                    //// modified by jgv, 2.11.01 for BUC61033 ////
                                    double U2A = (u2a < u2b) ? u2a : u2b;
                                    double U2B = (u2a < u2b) ? u2b : u2a;
                                    if (u2min == RealLast())
                                    {
                                        u2min = U2A;
                                        u2max = U2B;
                                    }
                                    else
                                    {
                                        if (Math.Abs(U2A - u2min) > PrecisionPConfusion)
                                        {
                                            ToBreak = true;
                                        }

                                        if (Math.Abs(U2B - u2max) > PrecisionPConfusion)
                                        {
                                            ToBreak = true;
                                        }
                                    }
                                    ///////////////////////////////////////////////
                                    if (newParam >= ParamMinOnLine && newParam <= ParamMaxOnLine)
                                    {
                                        SortAgain = true;
                                        SortIsOK = false;
                                        if (newParam > ponline)
                                        {
                                            if (u2a < u2b)
                                            {
                                                VTX.SetParameter(newParam);
                                            }
                                            else
                                            {
                                                VTXM1.SetParameter(newParam);
                                            }
                                        }
                                        else
                                        {
                                            if (u2a > u2b)
                                            {
                                                VTX.SetParameter(newParam);
                                            }
                                            else
                                            {
                                                VTXM1.SetParameter(newParam);
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                } //-- if(i!=j)
            }
        } while (!SortIsOK);

        //-- Recalcul de fipt et lapt
        //--
        nbvtx = NbVertex();
        if (nbvtx != 0)
        {
            do
            {
                SortIsOK = true;
                for (i = 2; i <= nbvtx; i++)
                {
                    if (svtx.Value(i - 1).ParameterOnLine() > svtx.Value(i).ParameterOnLine())
                    {
                        SortIsOK = false;
                        svtx.Exchange(i - 1, i);
                    }
                }
            } while (!SortIsOK);

            indl = nbvtx;
            indf = 1;
        }
    }

    // The removal of a duplicate on a restriction, with the first and last indices updated, that OCCT writes out four
    // times in the filters of ComputeVertexParameters.
    private void RemoveDuplicate(int theIndex, ref int theNbVtx)
    {
        svtx.Remove(theIndex);
        theNbVtx--;
        if (lapt)
        {
            if (indl > theIndex)
            {
                indl--;
            }
        }
        if (fipt)
        {
            if (indf > theIndex)
            {
                indf--;
            }
        }
    }
}
