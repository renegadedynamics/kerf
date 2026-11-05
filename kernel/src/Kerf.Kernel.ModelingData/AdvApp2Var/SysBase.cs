// Port of: src/ModelingData/TKGeomBase/AdvApp2Var/AdvApp2Var_SysBase.hxx
// Port of: src/ModelingData/TKGeomBase/AdvApp2Var/AdvApp2Var_SysBase.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Occt;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

namespace Kerf.Engine.Approximation.Advanced;

/// <summary>
/// The system layer of AdvApp2Var's f2c code: dynamic allocation of work arrays, memory fills, and the stubs of
/// the original Fortran library's message and debug routines (all of which do nothing in OCCT).
/// </summary>
/// <remarks>
/// <para>An instance holds the table of the allocations it made (<c>mcrgene_</c>) and their statistics
/// (<c>mcrstac_</c>), as in OCCT 8, where each routine needing work memory makes its own instance: the class is
/// reentrant. The common block <c>mblank__</c> is static, as in OCCT, and written only when the type is initialised
/// (decision 113).</para>
/// <para>Allocation (decision 114). The Fortran convention is that <c>mcrrqst_</c> allocates a zone and returns
/// an offset <c>IOFSET</c> from a caller's array <c>T</c> to it, so that <c>T(IOFSET + I)</c> addresses the zone.
/// A managed array can't be addressed from another one, so the port's <c>mcrrqst_</c> takes <c>T</c> by reference
/// and replaces it with a pointer to a new managed block laid out as OCCT's (an 8-byte start flag, the user zone
/// rounded up to 8 bytes, an 8-byte end flag), and sets <c>IOFSET</c> so that <c>T[IOFSET]</c> is the zone's
/// first element, as in OCCT. Callers keep indexing <c>T[IOFSET + I]</c> exactly as the C does. <c>mcrdelt_</c>
/// recognises the block by identity and the garbage collector frees it.</para>
/// </remarks>
internal sealed class SysBase
{
    public SysBase()
    {
        mainial_();
        mcrstac_ = new mcrstac_t();
    }

    // The C++ destructor asserts that every allocation was deleted (mcrgene_.ncore == 0); a managed block left
    // allocated is simply collected.

    /* Maximum number of allowed allocation requests.
       Currently the maximum known number of requests is 7 - see
       AdvApp2Var_MathBase::mmresol_(). So the current value is a safe margin and
       a reasonable balance to not provoke stack overflow (especially in
       multi-threaded execution). Previous number of 1000 was excessive but
       tolerable when used for static memory.
    */
    private const int MAX_ALLOC_NB = 32;

    private const int static_allocation = 0; /* indicates static allocation, currently not used */
    private const int heap_allocation = 1;   /* indicates heap allocation */

    // Value of the flags written before and after each allocated zone.
    private const double THE_FLAG = -134744073.0;

    /* Describes an individual memory allocation request.
       The addresses of OCCT are, in the port, the block (the managed array) and byte offsets in it (decision 114). */
    private struct mitem
    {
        public byte prot;
        public byte unit; // unit of allocation: 1, 2, 4 or 8
        public int reqsize;
        public Array? loc; // C++: reference address of the table (the port: the block it now points to)
        public int offset;
        public byte alloctype; // static_allocation or heap_allocation
        public int size;
        public Array? addr; // C++: address of the beginning of allocation (the port: the block)
        public int userzone; // not used
        public int startaddr; // C++: address of the start flag (the port: its byte offset in the block, or -1)
        public int endaddr; // C++: address of the end flag (the port: its byte offset in the block, or -1)
        public int rank;
    }

    private struct mcrgene_t
    {
        public mitem[]? icore; // [MAX_ALLOC_NB], allocated on the first request
        public int ncore;
        public byte lprot;
    }

    private mcrgene_t mcrgene_;

    /* Contains statistics on allocation requests.
       Index 0 corresponds to static_allocation, 1 - to heap allocation.
       nrqst - number of allocation requests;
       ndelt - number of deallocation requests;
       nbyte - current number of allocated bytes;
       mbyte - maximum number of ever allocated bytes.
    */
    private sealed class mcrstac_t
    {
        public readonly int[] nrqst = new int[2], ndelt = new int[2], nbyte = new int[2], mbyte = new int[2];
    }

    private readonly mcrstac_t mcrstac_;

    // The common block BLANK (reading/writing units and the debug level IBB) is left out: OCCT's macinit_ writes it and
    // nothing reads it (mnfndeb_ answers 0), unit 10I.

    //=================================================================================================

    /// <summary>
    /// Initialization of reading writing units and 'IBB'. IMODE: 0 = default (IMP 6, IBB 0, LEC 5), 1 = force IMP,
    /// 2 = force IBB, 3 = force LEC, to IVAL. Does nothing here: the common block it set is left out (nothing reads it).
    /// </summary>
    public static int macinit_(int imode, int ival)
    {
        return 0;
    } /* macinit__ */

    //=================================================================================================

    /// <summary>
    /// Require dynamic allocation of type INTEGER. If NBELEM &lt;= MAXELM (the size of ITABLO), IOFSET becomes 0 and
    /// ITABLO is used; otherwise a zone is allocated (see <see cref="mcrrqst_{T}"/>). IERCOD: 0 OK, 1 max nb of
    /// allocations attained, 2 incorrect arguments, 3 refused dynamic allocation.
    /// </summary>
    public int macrai4_(int nbelem, int maxelm, ref global::Kerf.Engine.Approximation.Advanced.F2cPtr<int> itablo, ref int iofset, ref int iercod)
    {
        int iunit;

        iunit = sizeof(int);
        /* Function Body */
        if (nbelem > maxelm)
        {
            /*AdvApp2Var_SysBase::*/
            mcrrqst_(iunit, nbelem, ref itablo, ref iofset, ref iercod);
        }
        else
        {
            iercod = 0;
            iofset = 0;
        }
        return 0;
    } /* macrai4_ */

    //=================================================================================================

    /// <summary>Demand of dynamic allocation of type DOUBLE PRECISION (as <see cref="macrai4_"/>).</summary>
    public int macrar8_(int nbelem, int maxelm, ref global::Kerf.Engine.Approximation.Advanced.F2cPtr<double> xtablo, ref int iofset, ref int iercod)
    {
        int c__8 = 8;

        /* Function Body */
        if (nbelem > maxelm)
        {
            /*AdvApp2Var_SysBase::*/
            mcrrqst_(c__8, nbelem, ref xtablo, ref iofset, ref iercod);
        }
        else
        {
            iercod = 0;
            iofset = 0;
        }
        return 0;
    } /* macrar8_ */

    //=================================================================================================

    private static int macrbrk_()
    {
        return 0;
    } /* macrbrk_ */

    //=================================================================================================

    /// <summary>Control of excesses of allocated memory zone: checks the flags around every allocation.</summary>
    private int macrchk_()
    {
        /* System generated locals */
        int i__1;

        /* Local variables */
        int i__, j;

        /* CONTROL OF FLAGS IN THE TABLE */
        i__1 = mcrgene_.ncore;
        for (i__ = 0; i__ < i__1; ++i__)
        {
            ref mitem item = ref mcrgene_.icore![i__];
            for (j = 0; j <= 1; ++j)
            {
                // pp: startaddr, then endaddr
                ref int pp = ref (j == 0 ? ref item.startaddr : ref item.endaddr);
                if (pp != -1)
                {
                    double t = ReadFlag(item.addr!, pp);
                    if (t != THE_FLAG)
                    {
                        /* MSG : '*** ERREUR  : REMOVAL FROM MEMORY OF ADDRESS
                           E:',ICORE(J,I) */
                        /*       AND OF RANK ICORE(12,I) */
                        macrerr_(pp, item.rank);

                        /* BACK-PARCING IN PHASE OF PRODUCTION */
                        maostrb_();

                        /* REMOVAL OF THE ADDRESS OF FLAG TO AVOID REMAKING ITS CONTROL */
                        pp = -1;
                    }
                }

                /* L100: */
            }

            /* L1000: */
        }
        return 0;
    } /* macrchk_ */

    //=================================================================================================

    private static int macrclw_(int iadfld, int iadflf, int nalloc)
    {
        _ = iadfld;
        _ = iadflf;
        _ = nalloc;
        return 0;
    } /* macrclw_ */

    //=================================================================================================

    /// <summary>Destruction of dynamic allocation of type INTEGER (made by <see cref="macrai4_"/>).</summary>
    public int macrdi4_(int nbelem, int maxelm, global::Kerf.Engine.Approximation.Advanced.F2cPtr<int> itablo, ref int iofset, ref int iercod)
    {
        _ = maxelm;
        int iunit;

        iunit = sizeof(int);
        /* Function Body */
        if (iofset != 0)
        {
            mcrdelt_(iunit, nbelem, itablo, ref iofset, ref iercod);
        }
        else
        {
            iercod = 0;
        }
        return 0;
    } /* macrdi4_ */

    //=================================================================================================

    /// <summary>Destruction of dynamic allocation of type DOUBLE PRECISION (made by <see cref="macrar8_"/>).</summary>
    public int macrdr8_(int nbelem, int maxelm, global::Kerf.Engine.Approximation.Advanced.F2cPtr<double> xtablo, ref int iofset, ref int iercod)
    {
        _ = maxelm;
        int c__8 = 8;

        /* Function Body */
        if (iofset != 0)
        {
            mcrdelt_(c__8, nbelem, xtablo, ref iofset, ref iercod);
        }
        else
        {
            iercod = 0;
        }
        return 0;
    } /* macrdr8_ */

    //=================================================================================================

    // Writing of address removed in allocs (OCCT writes nothing).
    private static int macrerr_(int iad, int nalloc)
    {
        _ = iad;
        _ = nalloc;
        return 0;
    } /* macrerr_ */

    //=================================================================================================

    /// <summary>
    /// Implementation of two flags start and end of the allocated zone, and setting to overflow of the user space in
    /// phase of production. IADFLD, IADFLF: byte offsets of the flags in the block; IPHASE: 0 official version,
    /// 1 production version; IZNUTI: size of the user zone in octets.
    /// </summary>
    private static int macrgfl_<T>(T[] block, int iadfld, int iadflf, int iphase, int iznuti)
        where T : unmanaged
    {
        /* original code used static int ifois=0 which served as static
           initialization long and was only used to call matrsym_() once; now
           this long is not used as matrsym_() always returns 0 and has no
           useful contents
        */
        int ifois = 1;

        int ibid = 0;
        int novfl = 0;

        if (ifois == 0)
        {
            matrsym_("NO_OVERFLOW", " ", ref novfl, ref ibid, 11L, 1L);
            ifois = 1;
        }

        /* CALCULATE THE OFFSET */
        Span<double> t = MemoryMarshal.Cast<byte, double>(MemoryMarshal.AsBytes(block.AsSpan()).Slice(iadfld));

        /*  SET TO OVERFLOW OF THE USER ZONE IN CASE OF PRODUCTION VERSION */
        if (iphase == 1 && novfl == 0)
        {
            int ienr = iznuti / 8;
            maoverf_(ienr, t.Slice(1));
        }

        /*  UPDATE THE START FLAG */
        t[0] = THE_FLAG;

        /*  FAKE CALL TO STOP THE DEBUGGER : */
        macrbrk_();

        /*  UPDATE THE START FLAG */
        MemoryMarshal.Cast<byte, double>(MemoryMarshal.AsBytes(block.AsSpan()).Slice(iadflf))[0] = THE_FLAG;

        /*  FAKE CALL TO STOP THE DEBUGGER : */
        macrbrk_();

        return 0;
    } /* macrgfl_ */

    // Reads the flag at a byte offset of a block (macrchk_'s *t).
    private static double ReadFlag(Array block, int byteOffset)
    {
        ReadOnlySpan<byte> bytes = block switch
        {
            double[] doubles => MemoryMarshal.AsBytes(doubles.AsSpan()),
            int[] ints => MemoryMarshal.AsBytes(ints.AsSpan()),
            _ => throw new ProgramErrorException("AdvApp2Var_SysBase: unexpected allocation type"),
        };
        return MemoryMarshal.Read<double>(bytes.Slice(byteOffset));
    }

    //=================================================================================================

    /// <summary>
    /// Messaging of routines of allocation (MCRRQST, MCRDELT, MCRLIST, CRINCR, CRPROT). OCCT has commented out the
    /// messages and writes nothing.
    /// </summary>
    private static int macrmsg_(string crout, int num, int it, double xt, string ct, long crout_len, long ct_len)
    {
        _ = crout;
        _ = num;
        _ = crout_len;
        _ = ct_len;

        /* Local variables */
        int inum;

        /*  READING OF THE LANGUAGE : */
        /* Parameter adjustments */
        _ = ct; // unused
        _ = xt; // unused
        _ = it; // unused

        /* Function Body */
        mamdlng_(3L);

        /*  INUM : TYPE OF MESSAGE  : 0 AS TEXT, 1 1 INTEGER TO BE WRITTEN */
        /*        -1 MESSAGE INEXISTING (1 INTEGER AND 1 CHAIN) */

        inum = -1;
        // The C++ table of messages (French, German, English) is commented out.

        /* --------------------------------------------------------------------- */
        /*  IMPLEMENTATION OF WRITE , WITH OR WITHOUT DATA : */

        if (inum == 0)
        {
        }
        else if (inum == 1)
        {
        }
        else
        {
            /*  MESSAGE DOES NOT EXIST ... */
        }

        return 0;
    } /* macrmsg_ */

    //=================================================================================================

    private static int macrstw_(int iadfld, int iadflf, int nalloc)
    {
        _ = iadfld;
        _ = iadflf;
        _ = nalloc;
        return 0;
    } /* macrstw_ */

    //=================================================================================================

    private static int madbtbk_(out int indice)
    {
        indice = 0;
        return 0;
    } /* madbtbk_ */

    //=================================================================================================

    /// <summary>Error message of a routine (OCCT writes nothing).</summary>
    public static int maermsg_(string cnompg, int icoder, long cnompg_len)
    {
        _ = cnompg;
        _ = icoder;
        _ = cnompg_len;
        return 0;
    } /* maermsg_ */

    //=================================================================================================

    /// <summary>
    /// Return translation of "name logic STRIM" in "internal syntax". IERCOD: 0 OK, 5 inexisting logic name,
    /// 6 translation too long, 7 critical error. OCCT keeps only the error logic.
    /// </summary>
    private static int magtlog_(string cnmlog, string chaine, ref int long__, ref int iercod, long cnmlog_len,
                                long chaine_len)
    {
        _ = chaine;
        _ = chaine_len;

        /* Local variables */
        int ibid = 0, ier = 0;

        long__ = 0;
        iercod = 0;

        /* CONTROL OF EXISTENCE OF THE LOGIC NAME */

        matrlog_(cnmlog, " ", ref ibid, ref ier, cnmlog_len, 255L);
        if (ier == 1)
        {
            goto L9500;
        }
        if (ier == 2)
        {
            goto L9700;
        }

        /* CONTROL OF THE LENGTH OF CHAIN */

        if (ibid > __i__len() /*chaine, chaine_len)*/)
        {
            goto L9600;
        }

        //__s__copy(chaine, cbid, chaine_len, ibid);
        long__ = ibid;

        goto L9999;

    /* ***********************************************************************/
    /*              ERROR PROCESSING                                         */
    /* ***********************************************************************/

    L9500:
        iercod = 5;
        //__s__copy(chaine, " ", chaine_len, 1L);
        goto L9999;

    L9600:
        iercod = 6;
        //__s__copy(chaine, " ", chaine_len, 1L);
        goto L9999;

    L9700:
        iercod = 7;
    //__s__copy(chaine, " ", chaine_len, 1L);

    /* ***********************************************************************/
    /*              RETURN TO THE CALLING PROGRAM                            */
    /* ***********************************************************************/

    L9999:
        return 0;
    } /* magtlog_ */

    //=================================================================================================

    public int mainial_()
    {
        mcrgene_.ncore = 0;
        mcrgene_.lprot = 0;
        return 0;
    } /* mainial_ */

    //=================================================================================================

    /// <summary>
    /// Initialization to a given value XVAL of a table of REAL*8 XTAB (from 1 to ITAILL). Calls MCRFILL, which moves
    /// by packs of 63 reals.
    /// </summary>
    public static int maitbr8_(int itaill, global::Kerf.Engine.Approximation.Advanced.F2cPtr<double> xtab, double xval)
    {
        int c__504 = 504;

        /* Initialized data */

        Span<double> buff0 = stackalloc double[63];

        /* System generated locals */
        int i__1;

        /* Local variables */
        int i__;
        Span<double> buffx = stackalloc double[63];
        int nbfois, noffst, nreste, nufois;

        /* Parameter adjustments */
        --xtab;

        /* Function Body */

        /* --------------------------------------------------------------------- */

        nbfois = itaill / 63;
        noffst = nbfois * 63;
        nreste = itaill - noffst;

        if (xval == 0.0)
        {
            if (nbfois >= 1)
            {
                i__1 = nbfois;
                for (nufois = 1; nufois <= i__1; ++nufois)
                {
                    mcrfill_(c__504, buff0, xtab + ((nufois - 1) * 63 + 1));
                    /* L1000: */
                }
            }

            if (nreste >= 1)
            {
                i__1 = nreste << 3;
                mcrfill_(i__1, buff0, xtab + (noffst + 1));
            }
        }
        else
        {
            for (i__ = 1; i__ <= 63; ++i__)
            {
                buffx[i__ - 1] = xval;
                /* L2000: */
            }
            if (nbfois >= 1)
            {
                i__1 = nbfois;
                for (nufois = 1; nufois <= i__1; ++nufois)
                {
                    mcrfill_(c__504, buffx, xtab + ((nufois - 1) * 63 + 1));
                    /* L3000: */
                }
            }

            if (nreste >= 1)
            {
                i__1 = nreste << 3;
                mcrfill_(i__1, buffx, xtab + (noffst + 1));
            }
        }

        /* --------------------------------------------------------------------- */

        return 0;
    } /* maitbr8_ */

    //=================================================================================================

    // Return the current language (OCCT returns nothing).
    private static int mamdlng_(long cmdlng_len)
    {
        _ = cmdlng_len;
        //__s__copy(cmdlng, macetat_.chlang, cmdlng_len, 4L);

        return 0;
    } /* mamdlng_ */

    //=================================================================================================

    private static int maostrb_()
    {
        return 0;
    } /* maostrb_ */

    //=================================================================================================

    // Refine trace-back in production phase.
    private static int maostrd_()
    {
        madbtbk_(out int imod);
        if (imod == 1)
        {
            maostrb_();
        }
        return 0;
    } /* maostrd_ */

    //=================================================================================================

    /// <summary>
    /// Initialisation in overflow (R8OVR) of a table of NBENTR DOUBLE PRECISION, by doubling copies of a buffer of 63.
    /// </summary>
    private static int maoverf_(int nbentr, global::Kerf.Engine.Approximation.Advanced.F2cPtr<double> dtable)
    {
        /* Initialized data */

        int ifois = 0;

        /* System generated locals */
        int i__1;

        /* Local variables */
        int ibid;
        Span<double> buff = stackalloc double[63];
        int ioct, indic, nrest, icompt;

        /* Parameter adjustments */
        --dtable;

        /* Function Body */

        /* vJMB R8OVR IS NOT YET initialized, so impossible to use DATA */
        /*         DATA BUFF / NLONGR * R8OVR / */

        /*    init of BUFF is done only once */

        if (ifois == 0)
        {
            for (icompt = 1; icompt <= 63; ++icompt)
            {
                buff[icompt - 1] = Data.Getmaovpar().r8ovr;
                /* L20: */
            }
            ifois = 1;
        }

        /* ^JMB */
        /* Exception */
        if (nbentr < 63)
        {
            nrest = nbentr << 3;
            mcrfill_(nrest, buff, dtable + 1);
        }
        else
        {
            /* Start & initialization */
            ioct = 504;
            mcrfill_(ioct, buff, dtable + 1);
            indic = 63;

            /* Loop. The upper limit is the int value of the logarithm of base 2
             */
            /* of NBENTR/NLONGR. */
            // std::log of a float is the float overload: MathF.Log.
            i__1 = (int)(MathF.Log((float)nbentr / (float)63.0) / MathF.Log((float)2.0));
            for (ibid = 1; ibid <= i__1; ++ibid)
            {
                mcrfill_(ioct, dtable + 1, dtable + (indic + 1));
                ioct += ioct;
                indic += indic;

                /* L10: */
            }

            nrest = (nbentr - indic) << 3;

            if (nrest > 0)
            {
                mcrfill_(nrest, dtable + 1, dtable + (indic + 1));
            }
        }
        return 0;
    } /* maoverf_ */

    //=================================================================================================

    public static int maovsr8_(ref int ivalcs)
    {
        ivalcs = Data.Getmaovpar().r8ncs;
        return 0;
    } /* maovsr8_ */

    //=================================================================================================

    private static int matrlog_(string cnmlog, string chaine, ref int length, ref int iercod, long cnmlog_len,
                                long chaine_len)
    {
        _ = cnmlog;
        _ = chaine;
        _ = cnmlog_len;
        _ = chaine_len;
        iercod = 1;
        length = 0;

        return 0;
    } /* matrlog_ */

    //=================================================================================================

    /// <summary>
    /// Return the value of a symbol defined during the initialisation of a user. IERCOD: 0 OK, 1 inexisting symbol,
    /// 2 other error.
    /// </summary>
    private static int matrsym_(string cnmsym, string chaine, ref int length, ref int iercod, long cnmsym_len,
                                long chaine_len)
    {
        _ = chaine;
        _ = chaine_len;

        /* SGI  CALL MAGTLOG (CNMSYM,CHAINE,LENGTH,IERCOD) */
        magtlog_(cnmsym, " ", ref length, ref iercod, cnmsym_len, 255L);
        /* SO...v */
        if (iercod == 5)
        {
            iercod = 1;
        }
        /* SO...^ */
        if (iercod >= 2)
        {
            iercod = 2;
        }
        // if (__s__cmp(chainx, "NONE", 255L, 4L) == 0) {
        if (__s__cmp() == 0)
        {
            //__s__copy(chainx, " ", 255L, 1L);
            length = 0;
        }
        //__s__copy(chaine, chainx, chaine_len, 255L);
        /* SGI...^ */

        /* L9999: */
        return 0;
    } /* matrsym_ */

    //=================================================================================================

    /// <summary>
    /// Dynamic allocation on common. KOP: 1 allocation, 2 destruction; NOCT: number of octets; IADR: address of the
    /// first octet (the port: the index of the first double in DTAB); IER: 0 OK, 1 to 4 errors.
    /// </summary>
    /// <remarks>Unused: OCCT allocates on the heap only (<c>mcrrqst_</c>). ITAB and NTAB are not saved between two
    /// calls, as OCCT notes.</remarks>
    private static int mcrcomm_(int kop, int noct, ref int iadr, ref int ier)
    {
        /* Initialized data */

        int ntab = 0;

        /* System generated locals */
        int i__1, i__2;

        /* Local variables */
        int ideb;
        // C++: NCollection_Array1<double> dtab(0, 32000 - 1), of which only addresses are used.
        NCollection_Array1<int> itab = new(0, 160 - 1); // was [4][40] but now flattened to a single array
        int ipre;
        int i__, j, k;

        ier = 0;

        /*  ALLOCATION : FIND A HOLE */

        if (kop == 1)
        {
            iadr = 0;
            if (noct < 1)
            {
                ier = 1;
                goto L9900;
            }
            if (ntab >= 40)
            {
                ier = 2;
                goto L9900;
            }

            i__1 = ntab + 1;
            for (i__ = 1; i__ <= i__1; ++i__)
            {
                if (i__ <= 1)
                {
                    ipre = 1;
                }
                else
                {
                    ipre = itab[((i__ - 1) << 2) - 3] + itab[((i__ - 1) << 2) - 4];
                }
                if (i__ <= ntab)
                {
                    ideb = itab[(i__ << 2) - 3];
                }
                else
                {
                    ideb = 32001;
                }
                if ((ideb - ipre) << 3 >= noct)
                {
                    /*  A HOLE WAS FOUND */
                    i__2 = i__;
                    for (j = ntab; j >= i__2; --j)
                    {
                        for (k = 1; k <= 4; ++k)
                        {
                            itab[k + ((j + 1) << 2) - 5] = itab[k + (j << 2) - 5];
                            /* L1003: */
                        }
                        /* L1002: */
                    }
                    ++ntab;
                    itab[(i__ << 2) - 4] = noct / 8 + 1;
                    itab[(i__ << 2) - 3] = ipre;
                    itab[(i__ << 2) - 2] = noct;
                    iadr = ipre - 1; // C++: &dtab[ipre - 1]
                    itab[(i__ << 2) - 1] = iadr;
                    goto L9900;
                }
                /* L1001: */
            }

            /*  NO HOLE */

            ier = 3;
            goto L9900;

            /* ----------------------------------- */
            /*  DESTRUCTION OF THE ALLOCATION NUM : */
        }
        else
        {
            i__1 = ntab;
            for (i__ = 1; i__ <= i__1; ++i__)
            {
                if (noct != itab[(i__ << 2) - 2])
                {
                    goto L2001;
                }
                if (iadr != itab[(i__ << 2) - 1])
                {
                    goto L2001;
                }
                /*  THE ALLOCATION TO BE REMOVED WAS FOUND */
                i__2 = ntab;
                for (j = i__ + 1; j <= i__2; ++j)
                {
                    for (k = 1; k <= 4; ++k)
                    {
                        itab[k + ((j - 1) << 2) - 5] = itab[k + (j << 2) - 5];
                        /* L2003: */
                    }
                    /* L2002: */
                }
                --ntab;
                goto L9900;
            L2001:;
            }

            /*        THE ALLOCATION DOES NOT EXIST */

            ier = 4;
            /* PP         GOTO 9900 */
        }

    L9900:
        return 0;
    } /* mcrcomm_ */

    //=================================================================================================

    /// <summary>
    /// Destruction of a dynamic allocation made by <see cref="mcrrqst_{T}"/>, with the same arguments. IERCOD: 0 OK,
    /// 1 pb of de-allocation of a zone allocated in common, 2 the system refuses, 3 the allocation does not exist.
    /// When the allocation is destroyed, IOFSET is set to int.MaxValue, so that a later access through it traps.
    /// </summary>
    public int mcrdelt_<T>(int iunit, int isize, global::Kerf.Engine.Approximation.Advanced.F2cPtr<T> t, ref int iofset, ref int iercod)
        where T : unmanaged
    {
        int ier = 0;
        int noct, iver, ksys, i__, n, nrang, ibyte;
        int iadfd, iadff, iaddr = 0;
        int kop;

        /* Function Body */
        iercod = 0;

        /* SEARCH IN MCRGENE */

        n = -1;

        for (i__ = mcrgene_.ncore - 1; i__ >= 0; --i__)
        {
            ref mitem item = ref mcrgene_.icore![i__];
            if (iunit == item.unit && isize == item.reqsize && SameBlock(t, item.loc)
                && iofset == item.offset)
            {
                n = i__;
                goto L1100;
            }
            /* L1001: */
        }
    L1100:

        /* IF THE ALLOCATION DOES NOT EXIST, LEAVE */

        if (n < 0)
        {
            goto L9003;
        }

        /* ALLOCATION RECOGNIZED : RETURN OTHER INFOS */

        ksys = mcrgene_.icore![n].alloctype;
        ibyte = mcrgene_.icore[n].size;
        iadfd = mcrgene_.icore[n].startaddr;
        iadff = mcrgene_.icore[n].endaddr;
        nrang = mcrgene_.icore[n].rank;

        /*     Control of flags */

        madbtbk_(out iver);
        if (iver == 1)
        {
            macrchk_();
        }

        if (ksys == static_allocation)
        {
            /* DE-ALLOCATION ON COMMON */
            kop = 2;
            mcrcomm_(kop, ibyte, ref iaddr, ref ier);
            if (ier != 0)
            {
                goto L9001;
            }
        }
        else
        {
            /* DE-ALLOCATION SYSTEM */
            mcrfree_(ibyte, mcrgene_.icore[n].addr, ref ier);
            if (ier != 0)
            {
                goto L9002;
            }
        }

        /* CALL ALLOWING TO CANCEL AUTOMATIC WATCH BY THE DEBUGGER */

        macrclw_(iadfd, iadff, nrang);

        /* UPDATE OF STATISTICS */
        ++mcrstac_.ndelt[ksys];
        mcrstac_.nbyte[ksys] -= mcrgene_.icore[n].unit * mcrgene_.icore[n].reqsize;

        /* REMOVAL OF PARAMETERS IN MCRGENE */
        if (n < MAX_ALLOC_NB - 1)
        {
            noct = mcrgene_.ncore - (n + 1); // C++: in bytes, copied with mcrfill_
            Array.Copy(mcrgene_.icore, n + 1, mcrgene_.icore, n, noct);
        }
        --mcrgene_.ncore;

        /* *** Set to overflow of IOFSET */
        {
            /* assign max positive int to *iofset */
            iofset = int.MaxValue; // C++: the maximum intptr_t (2147483647 for 32bit)
        }
        goto L9900;

    /* --------------------------------------------------------------------- */
    /*     ERROR PROCESSING */

    L9001:
        /*  REFUSE DE-ALLOCATION BY ROUTINE 'MCRCOMM' (ALLOC DS COMMON) */
        iercod = 1;
        maermsg_("MCRDELT", iercod, 7L);
        maostrd_();
        goto L9900;

    /*  REFUSE DE-ALLOCATION BY THE SYSTEM */
    L9002:
        iercod = 2;
        maermsg_("MCRDELT", iercod, 7L);
        macrmsg_("MCRDELT", iercod, 0, 0.0, " ", 7L, 1L);
        maostrd_();
        goto L9900;

    /* ALLOCATION DOES NOT EXIST */
    L9003:
        iercod = 3;
        maermsg_("MCRDELT", iercod, 7L);
        maostrd_();
        goto L9900;

    L9900:

        return 0;
    } /* mcrdelt_ */

    // The C++ `loc == mcrgene_.icore[i__].loc`: whether the pointer T is the block (decision 114).
    private static bool SameBlock<T>(global::Kerf.Engine.Approximation.Advanced.F2cPtr<T> t, Array? block)
        where T : unmanaged
    {
        return block is T[] aBlock && aBlock.Length > 0
               && Unsafe.AreSame(ref t.UnderlyingReference, ref MemoryMarshal.GetArrayDataReference(aBlock));
    }

    //=================================================================================================

    /// <summary>
    /// Transfer a memory zone of SIZE octets into another, managing intersections (as memmove). The port copies
    /// arrays of one element type; SIZE stays in octets.
    /// </summary>
    public static int mcrfill_(int size, global::Kerf.Engine.Approximation.Advanced.F2cConstPtr<double> tin, global::Kerf.Engine.Approximation.Advanced.F2cPtr<double> tout) => mcrfill_core(size, tin, tout);

    /// <summary>Transfer a memory zone of SIZE octets into another, as <see cref="mcrfill_(int, F2cConstPtr{double}, F2cPtr{double})"/>.</summary>
    public static int mcrfill_(int size, global::Kerf.Engine.Approximation.Advanced.F2cConstPtr<int> tin, global::Kerf.Engine.Approximation.Advanced.F2cPtr<int> tout) => mcrfill_core(size, tin, tout);

    private static int mcrfill_core<T>(int size, global::Kerf.Engine.Approximation.Advanced.F2cConstPtr<T> tin, global::Kerf.Engine.Approximation.Advanced.F2cPtr<T> tout)
        where T : unmanaged
    {
        int count = (size + Unsafe.SizeOf<T>() - 1) / Unsafe.SizeOf<T>();
        ReadOnlySpan<byte> jmin = MemoryMarshal.AsBytes(tin.AsSpan(count)).Slice(0, size);
        Span<byte> jmout = MemoryMarshal.AsBytes(tout.AsSpan(count)).Slice(0, size);
        // The C++ copies with memcpy when the zones don't overlap, and byte by byte forwards or backwards otherwise:
        // the result is memmove's, which Span.CopyTo is.
        jmin.CopyTo(jmout);
        return 0;
    }

    //=================================================================================================

    // Desallocation of a memory zone: the garbage collector frees the block.
    private static int mcrfree_(int ibyte, Array? iadr, ref int ier)
    {
        _ = ibyte;
        _ = iadr;
        ier = 0;
        return 0;
    }

    //=================================================================================================

    // Demand of memory allocation: a block of SZ bytes of the element type (OCCT's Standard::Allocate). As OCCT's
    // allocator, an allocation failure throws (OutOfMemoryException) rather than setting IER = 1.
    private static int mcrgetv_<T>(int sz, out T[] iad, ref int ier)
        where T : unmanaged
    {
        ier = 0;
        iad = new T[sz / Unsafe.SizeOf<T>()];
        return 0;
    }

    //=================================================================================================

    /// <summary>Print table of current dynamic allocations (OCCT writes nothing).</summary>
    private int mcrlist_(ref int ier)
    {
        /* System generated locals */
        int i__1;

        /* Local variables */
        string cfmt = "";
        double dfmt = 0.0;
        int ifmt, i__, nufmt, ntotal;
        string subrou = "MCRLIST";

        ier = 0;

        /*     WRITE HEADING */

        nufmt = 1;
        ifmt = mcrgene_.ncore;
        macrmsg_(subrou, nufmt, ifmt, dfmt, cfmt, 7L, 1L);

        ntotal = 0;

        i__1 = mcrgene_.ncore;
        for (i__ = 0; i__ < i__1; ++i__)
        {
            nufmt = 2;
            ifmt = mcrgene_.icore![i__].unit * mcrgene_.icore[i__].reqsize;
            macrmsg_(subrou, nufmt, ifmt, dfmt, cfmt, 7L, 1L);
            ntotal += ifmt;
            /* L1001: */
        }

        nufmt = 3;
        ifmt = ntotal;
        macrmsg_(subrou, nufmt, ifmt, dfmt, cfmt, 7L, 1L);

        return 0;
    } /* mcrlist_ */

    //=================================================================================================

    /// <summary>
    /// Implementation of dynamic memory allocation of ISIZE units of IUNIT octets (1, 2, 4 or 8; the port also
    /// requires the size of T). On return T points to the new zone's block, and T[IOFSET + I], I from 0, is its
    /// I-th unit (decision 114). IERCOD: 0 OK, 1 max nb of allocs reached, 2 arguments incorrect, 3 refused
    /// dynamic allocation.
    /// </summary>
    public int mcrrqst_<T>(int iunit, int isize, ref global::Kerf.Engine.Approximation.Advanced.F2cPtr<T> t, ref int iofset, ref int iercod)
        where T : unmanaged
    {
        int i__1, i__2;

        /* Local variables */
        double dfmt = 0.0;
        int ifmt, iver;
        string subr = "MCRRQST";
        int ksys, ibyte, irest, ier = 0;
        int iadfd, iadff, lofset;
        T[] iaddr;
        int izu;

        /* Function Body */
        iercod = 0;

        if (mcrgene_.ncore >= MAX_ALLOC_NB)
        {
            goto L9001;
        }
        if (iunit != 1 && iunit != 2 && iunit != 4 && iunit != 8)
        {
            goto L9002;
        }
        // The port: the zone is an array of T, so the unit must be T's size.
        if (iunit != Unsafe.SizeOf<T>())
        {
            goto L9002;
        }

        /* Calculate the size required by the user */
        ibyte = iunit * isize;

        /* Find the type of version (Phase of Production or Version Client) */
        madbtbk_(out iver);

        /* Control allocated size in Production phase */

        if (iver == 1)
        {
            if (ibyte == 0)
            {
                // do__lio(&c__9, &c__1, "Require zero allocation", 26L);
                maostrb_();
            }
            else if (ibyte >= 4096000)
            {
                // do__lio(&c__9, &c__1, "Require allocation above 4 Mega-Octets : ", 50L);
                maostrb_();
            }
        }

        /* CALCULATE THE SIZE OF THE USER ZONE (IZU) */
        /*     . add size required by the user (IBYTE) */
        /*     . add delta for alinement with the base (the port: none, a block is aligned) */
        /*     . round to multiple of 8 above */

        izu = ibyte;
        irest = izu % 8;
        if (irest != 0)
        {
            izu = izu + 8 - irest;
        }

        /* CALCULATE THE SIZE REQUIRED FROM THE PRIMITIVE OF ALLOC */
        /*     . add size of the user zone */
        /*     . add 8 for alinement of start address of */
        /*       allocation on multiple of 8 (the port: not needed, but counted in the statistics as by OCCT) */
        /*     . add 16 octets for two flags */

        ibyte = izu + 24;

        /* DEMAND OF ALLOCATION */

        ksys = heap_allocation;
        mcrgetv_(izu + 16, out iaddr, ref ier);
        if (ier != 0)
        {
            goto L9003;
        }

        /* CALCULATE THE ADDRESSES OF FLAGS (byte offsets in the block) */

        iadfd = 0;
        iadff = iadfd + 8 + izu;

        /* CALCULATE USER OFFSET : */
        /*     . difference between the user start address and the */
        /*       base address */
        /*     . converts this difference in the user unit */

        lofset = iadfd + 8;
        iofset = lofset / iunit;
        t = iaddr;

        /* If phase of production control flags */
        if (iver == 1)
        {
            macrchk_();
        }

        /*     SET FLAGS */
        /*     . the first long is set by IADFD and the second by IADFF */
        /*     . if phase of production, set to overflow the ZU */
        macrgfl_(iaddr, iadfd, iadff, iver, izu);

        /* RANGING OF PARAMETERS IN MCRGENE */

        mcrgene_.icore ??= new mitem[MAX_ALLOC_NB];
        ref mitem item = ref mcrgene_.icore[mcrgene_.ncore];
        item.prot = mcrgene_.lprot;
        item.unit = (byte)iunit;
        item.reqsize = isize;
        item.loc = iaddr;
        item.offset = iofset;
        item.alloctype = (byte)ksys;
        item.size = ibyte;
        item.addr = iaddr;
        item.userzone = mcrgene_.ncore;
        item.startaddr = iadfd;
        item.endaddr = iadff;
        item.rank = mcrgene_.ncore + 1;
        ++mcrgene_.ncore;

        mcrgene_.lprot = 0;

        /* CALL ALLOWING AUTOIMPLEMENTATION OF THE SET WATCH BY THE DEBUGGER */

        macrstw_(iadfd, iadff, mcrgene_.ncore);

        /* STATISTICS */

        ++mcrstac_.nrqst[ksys];
        mcrstac_.nbyte[ksys] +=
            mcrgene_.icore[mcrgene_.ncore - 1].unit * mcrgene_.icore[mcrgene_.ncore - 1].reqsize;
        /* Computing MAX */
        i__1 = mcrstac_.mbyte[ksys];
        i__2 = mcrstac_.nbyte[ksys];
        mcrstac_.mbyte[ksys] = Max(i__1, i__2);

        goto L9900;

    /* ----------------------------------------------------------------------*
     */
    /*  ERROR PROCESSING */

    /*  MAX NB OF ALLOC REACHED : */
    L9001:
        iercod = 1;
        ifmt = MAX_ALLOC_NB;
        macrmsg_(subr, iercod, ifmt, dfmt, " ", 7L, 1L);
        maostrd_();
        goto L9900;

    /*  INCORRECT ARGUMENTS */
    L9002:
        iercod = 2;
        ifmt = iunit;
        macrmsg_(subr, iercod, ifmt, dfmt, " ", 7L, 1L);
        goto L9900;

    /* SYSTEM REFUSES ALLOCATION */
    L9003:
        iercod = 3;
        ifmt = ibyte;
        macrmsg_(subr, iercod, ifmt, dfmt, " ", 7L, 1L);
        maostrd_();
        mcrlist_(ref ier);
        goto L9900;

    /* --------------------------------------------------------------------- */

    L9900:
        mcrgene_.lprot = 0;
        return 0;
    } /* mcrrqst_ */

    //=================================================================================================

    /// <summary>Entry message of a routine (OCCT writes nothing).</summary>
    public static int mgenmsg_(string nomprg, long nomprg_len)
    {
        _ = nomprg;
        _ = nomprg_len;
        return 0;
    } /* mgenmsg_ */

    //=================================================================================================

    /// <summary>Exit message of a routine (OCCT writes nothing).</summary>
    public static int mgsomsg_(string nomprg, long nomprg_len)
    {
        _ = nomprg;
        _ = nomprg_len;
        return 0;
    } /* mgsomsg_ */

    //=================================================================================================

    /// <summary>Reset to zero TAILLE octets of a table of logic or integer.</summary>
    public static void miraz_(int taille, global::Kerf.Engine.Approximation.Advanced.F2cPtr<double> adt) => miraz_core(taille, adt);

    /// <summary>Reset to zero TAILLE octets of a table of integers.</summary>
    public static void miraz_(int taille, global::Kerf.Engine.Approximation.Advanced.F2cPtr<int> adt) => miraz_core(taille, adt);

    private static void miraz_core<T>(int taille, global::Kerf.Engine.Approximation.Advanced.F2cPtr<T> adt)
        where T : unmanaged
    {
        int count = (taille + Unsafe.SizeOf<T>() - 1) / Unsafe.SizeOf<T>();
        MemoryMarshal.AsBytes(adt.AsSpan(count)).Slice(0, taille).Clear();
    }

    //=================================================================================================

    /// <summary>The debug level (always 0).</summary>
    public static int mnfndeb_()
    {
        int ret_val;
        ret_val = 0;
        return ret_val;
    } /* mnfndeb_ */

    //=================================================================================================

    /// <summary>Transfer NBINTG integers from one zone to another.</summary>
    public static int msifill_(int nbintg, global::Kerf.Engine.Approximation.Advanced.F2cConstPtr<int> ivecin, global::Kerf.Engine.Approximation.Advanced.F2cPtr<int> ivecou)
    {
        int nocte;

        /* Parameter adjustments */
        --ivecou;
        --ivecin;

        /* Function Body */
        nocte = nbintg * sizeof(int);
        mcrfill_(nocte, ivecin + 1, ivecou + 1);
        return 0;
    } /* msifill_ */

    //=================================================================================================

    /// <summary>Transfer NBREEL reals from one zone to another.</summary>
    public static int msrfill_(int nbreel, global::Kerf.Engine.Approximation.Advanced.F2cConstPtr<double> vecent, global::Kerf.Engine.Approximation.Advanced.F2cPtr<double> vecsor)
    {
        int nocte;

        /* Parameter adjustments */
        --vecsor;
        --vecent;

        /* Function Body */
        nocte = nbreel * sizeof(double);
        mcrfill_(nocte, vecent + 1, vecsor + 1);
        return 0;
    } /* msrfill_ */

    //=================================================================================================

    /// <summary>Write message on console alpha if IBB &gt; 0 (OCCT writes nothing).</summary>
    public static int mswrdbg_(string ctexte, long ctexte_len)
    {
        _ = ctexte;
        _ = ctexte_len;

        if (mnfndeb_() >= 1)
        {
            // do__lio(&c__9, &c__1, "Dbg ", 4L);
            // do__lio(&c__9, &c__1, ctexte, ctexte_len);
        }
        return 0;
    } /* mswrdbg_ */

    private static int __i__len()
    {
        return 0;
    }

    private static int __s__cmp()
    {
        return 0;
    }

    //=================================================================================================

    /// <summary>Reset to zero a table of TAILLE DOUBLE PRECISION (TAILLE * 8 octets, whatever the element type).</summary>
    public static void mvriraz_(int taille, global::Kerf.Engine.Approximation.Advanced.F2cPtr<double> adt)
    {
        int offset;
        offset = taille * 8;
        miraz_(offset, adt);
    }
}
